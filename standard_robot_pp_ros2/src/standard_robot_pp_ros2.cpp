// Copyright 2025 SMBU-PolarBear-Robotics-Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "standard_robot_pp_ros2/standard_robot_pp_ros2.hpp"

#include <memory>

#include "standard_robot_pp_ros2/crc8_crc16.hpp"
#include "standard_robot_pp_ros2/packet_typedef.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#define USB_NOT_OK_SLEEP_TIME 1000   // (ms)
#define USB_PROTECT_SLEEP_TIME 1000  // (ms)

using namespace std::chrono_literals;

namespace standard_robot_pp_ros2
{



  
StandardRobotPpRos2Node::StandardRobotPpRos2Node(const rclcpp::NodeOptions & options)
: Node("StandardRobotPpRos2Node", options),  // 初始化ROS2节点，节点名称为"StandardRobotPpRos2Node"
  owned_ctx_{new IoContext(2)},              // 创建IO上下文，使用2个线程处理异步IO
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}  // 创建串口驱动实例
{
  RCLCPP_INFO(get_logger(), "Start StandardRobotPpRos2Node!");  // 记录节点启动日志

  // 获取参数配置
  getParams();
  // 创建ROS2发布者
  createPublisher();
  // 创建ROS2订阅者
  createSubscription();

  // 初始化机器人底盘型号映射表 (ID -> 型号名称)
  robot_models_.chassis = {
    {0, "无底盘"}, {1, "麦轮底盘"}, {2, "全向轮底盘"}, {3, "舵轮底盘"}, {4, "平衡底盘"}};
  // 初始化机器人云台型号映射表
  robot_models_.gimbal = {{0, "无云台"}, {1, "yaw_pitch直连云台"}};
  // 初始化机器人发射机构型号映射表
  robot_models_.shoot = {{0, "无发射机构"}, {1, "摩擦轮+拨弹盘"}, {2, "气动+拨弹盘"}};
  // 初始化机器人机械臂型号映射表
  robot_models_.arm = {{0, "无机械臂"}, {1, "mini机械臂"}};
  // 初始化自定义控制器型号映射表
  robot_models_.custom_controller = {{0, "无自定义控制器"}, {1, "mini自定义控制器"}};

  // 启动串口保护线程，负责串口连接维护和异常恢复
  serial_port_protect_thread_ = std::thread(&StandardRobotPpRos2Node::serialPortProtect, this);
  // 启动数据接收线程，负责从串口读取并解析数据
  receive_thread_ = std::thread(&StandardRobotPpRos2Node::receiveData, this);
  // 启动数据发送线程，负责向串口发送控制指令
  send_thread_ = std::thread(&StandardRobotPpRos2Node::sendData, this);
}






StandardRobotPpRos2Node::~StandardRobotPpRos2Node()
{
  // 等待发送线程结束
  if (send_thread_.joinable()) {
    send_thread_.join();
  }

  // 等待接收线程结束
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  // 等待串口保护线程结束
  if (serial_port_protect_thread_.joinable()) {
    serial_port_protect_thread_.join();
  }

  // 关闭串口
  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  // 等待IO上下文退出
  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}


/**
 * @brief 从小端序字节向量中读取指定类型的数值
 */
template <typename T>
T readLittleEndian(const std::vector<uint8_t> & payload, size_t offset)
{
  T value{};
  if (offset + sizeof(T) > payload.size()) {
    return value;
  }
  std::memcpy(&value, payload.data() + offset, sizeof(T));
  return value;
}

/**
 * @brief 以小端序将数值追加到字节向量中（用于发送）
 */
template <typename T>
void appendLittleEndian(std::vector<uint8_t> & payload, T value)
{
  const auto * raw = reinterpret_cast<const uint8_t *>(&value);
  payload.insert(payload.end(), raw, raw + sizeof(T));
}



void StandardRobotPpRos2Node::createPublisher()
{
  // 创建IMU数据发布者，发布传感器IMU数据到serial/receive话题
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("serial/receive", 10);
  // 创建机器人状态信息发布者，发布机器人状态信息到serial/robot_state_info话题
  robot_state_info_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::RobotStateInfo>("serial/robot_state_info", 10);
  // 创建关节状态发布者，发布云台关节状态到serial/gimbal_joint_state话题
  joint_state_pub_ =
    this->create_publisher<sensor_msgs::msg::JointState>("serial/gimbal_joint_state", 10);
  // 创建机器人运动信息发布者，发布机器人运动数据到serial/robot_motion话题
  robot_motion_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("serial/robot_motion", 10);
  // 创建赛事事件数据发布者，发布赛事事件数据到referee/event_data话题
  event_data_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::EventData>("referee/event_data", 10);
  // 创建全场机器人血量发布者，发布所有机器人血量信息到referee/all_robot_hp话题
  all_robot_hp_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::GameRobotHP>("referee/all_robot_hp", 10);
  // 创建比赛状态发布者，发布比赛状态信息到referee/game_status话题
  game_status_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::GameStatus>("referee/game_status", 10);
  // 创建地面机器人位置发布者，发布地面机器人位置到referee/ground_robot_position话题
  ground_robot_position_pub_ = this->create_publisher<pb_rm_interfaces::msg::GroundRobotPosition>(
    "referee/ground_robot_position", 10);
  // 创建RFID状态发布者，发布RFID模块状态到referee/rfid_status话题
  rfid_status_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::RfidStatus>("referee/rfid_status", 10);
  // 创建机器人状态发布者，发布机器人详细状态到referee/robot_status话题
  robot_status_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::RobotStatus>("referee/robot_status", 10);
  // 创建增益状态发布者，发布增益效果状态到referee/buff话题
  buff_pub_ = this->create_publisher<pb_rm_interfaces::msg::Buff>("referee/buff", 10);
  // 创建哨兵状态发布者，发布哨兵状态到serial/sentry_pose话题
  sentry_pose_pub_ = this->create_publisher<example_interfaces::msg::UInt8>("serial/sentry_pose", 10);
}











// 创建新的调试数据发布者
// 该函数用于动态创建调试数据发布者，将调试数据发布到 "serial/debug/<name>" 话题
// 主要用于发布来自下位机的实时调试数值（如PID参数、传感器原始数据等），方便在ROS2端进行可视化和监控
void StandardRobotPpRos2Node::createNewDebugPublisher(const std::string & name)
{
  RCLCPP_INFO(get_logger(), "Create new debug publisher: %s", name.c_str());
  // 构造调试话题名称，格式为 serial/debug/<调试名称>
  std::string topic_name = "serial/debug/" + name;
  // 创建Float64类型的发布者，用于发布单个浮点数值
  auto debug_pub = this->create_publisher<example_interfaces::msg::Float64>(topic_name, 10);
  // 将发布者存入映射表，以便后续根据名称查找和使用
  debug_pub_map_.insert(std::make_pair(name, debug_pub));
}






// 创建ROS2订阅者
void StandardRobotPpRos2Node::createSubscription()
{
  // 创建底盘速度控制订阅者，订阅cmd_vel话题
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&StandardRobotPpRos2Node::cmdVelCallback, this, std::placeholders::_1));

  // 创建云台关节控制订阅者，订阅cmd_gimbal_joint话题
  // cmd_gimbal_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
  //   "cmd_gimbal_joint", 10,
  //   std::bind(&StandardRobotPpRos2Node::cmdGimbalJointCallback, this, std::placeholders::_1));

  // 创建射击控制订阅者，订阅cmd_shoot话题
  // cmd_shoot_sub_ = this->create_subscription<example_interfaces::msg::UInt8>(
  //   "cmd_shoot", 10,
  //   std::bind(&StandardRobotPpRos2Node::cmdShootCallback, this, std::placeholders::_1));
  // 创建视觉追踪目标订阅者，订阅tracker/target话题
  // cmd_tracking_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
  //   "tracker/target", 10,
  //   std::bind(&StandardRobotPpRos2Node::visionTargetCallback, this, std::placeholders::_1));
}









// 获取ROS2参数配置
void StandardRobotPpRos2Node::getParams()
{
  // 使用类型别名简化串口配置类型的书写
  // 流控制类型：用于串口通信的流控制设置（无/硬件/软件流控）
  // 使用的是 drivers::serial_driver 库，这是 ROS2 的 serial_driver 包提供的串口驱动库
  // 该库封装了跨平台的串口通信功能，支持配置波特率、流控制、校验位、停止位等参数
  using FlowControl = drivers::serial_driver::FlowControl;
  // 校验位类型：用于串口数据校验（无校验/奇校验/偶校验）
  using Parity = drivers::serial_driver::Parity;
  // 停止位类型：用于串口帧结束标识（1位/1.5位/2位停止位）
  using StopBits = drivers::serial_driver::StopBits;

  // 初始化串口配置参数
  uint32_t baud_rate{};           // 波特率
  auto fc = FlowControl::NONE;    // 流控制，默认为无
  auto pt = Parity::NONE;         // 校验位，默认为无校验
  auto sb = StopBits::ONE;        // 停止位，默认为1位

  // 获取设备名称参数
  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  // 获取波特率参数
  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  // 获取流控制参数并转换为枚举类型
  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  // 获取校验位参数并转换为枚举类型
  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    // 校验位：用于串口通信中检测数据传输错误的校验方式
    // none - 无校验：不添加校验位，传输效率最高但无错误检测能力
    // odd - 奇校验：确保数据位中1的个数加上校验位后为奇数，可检测单比特错误
    // even - 偶校验：确保数据位中1的个数加上校验位后为偶数，可检测单比特错误
    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }



  // 获取停止位参数并转换为枚举类型
  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    // 停止位：用于串口帧结束标识（1位/1.5位/2位停止位）
    // 1 - 1位停止位：每个字符后面跟着1位停止位，用于分隔字符
    // 1.5 - 1.5位停止位：每个字符后面跟着1.5位停止位，用于分隔字符
    // 2 - 2位停止位：每个字符后面跟着2位停止位，用于分隔字符
    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }






  // 创建串口配置对象，使用智能指针管理内存
  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);

  // 获取其他功能开关参数
  record_rosbag_ = declare_parameter("record_rosbag", false);           // 是否录制rosbag
  set_detector_color_ = declare_parameter("set_detector_color", false); // 是否设置检测器颜色
  debug_ = declare_parameter("debug", false);                           // 是否开启调试模式
}

























/********************************************************/
/* Serial port protect                                  */
/********************************************************/
void StandardRobotPpRos2Node::serialPortProtect()
{
  RCLCPP_INFO(get_logger(), "Start serialPortProtect!");

  // @TODO: 1.保持串口连接 2.串口断开重连 3.串口异常处理

  // 初始化串口，配置串口设备名称和参数
  serial_driver_->init_port(device_name_, *device_config_);
  // 尝试打开串口
  try {
    // 检查串口是否未打开，若未打开则执行打开操作
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      is_usb_ok_ = true;
      RCLCPP_INFO(get_logger(), "Serial port opened!");

    }
  } catch (const std::exception & ex) {
    // 打开串口失败，记录错误日志并设置状态标志
    RCLCPP_ERROR(get_logger(), "Open serial port failed : %s", ex.what());
    is_usb_ok_ = false;
  }

  // 设置串口状态为正常
  is_usb_ok_ = true;
  // 线程休眠，等待串口稳定
  std::this_thread::sleep_for(std::chrono::milliseconds(USB_PROTECT_SLEEP_TIME));

  // 主循环，持续监控串口状态
  while (rclcpp::ok()) {
    // 检查串口是否异常
    if (!is_usb_ok_) {
      try {
        // 如果串口处于打开状态，先关闭串口
        if (serial_driver_->port()->is_open()) {
          serial_driver_->port()->close();
        }

        // 重新打开串口
        serial_driver_->port()->open();

        // 检查串口是否成功打开
        if (serial_driver_->port()->is_open()) {
          RCLCPP_INFO(get_logger(), "Serial port opened!");
          is_usb_ok_ = true;
        }
      } catch (const std::exception & ex) {
        // 重连失败，设置状态标志并记录错误日志
        is_usb_ok_ = false;
        RCLCPP_ERROR(get_logger(), "Open serial port failed : %s", ex.what());
      }
    }
    // 线程休眠，进入下一次检测周期
    // 作用：控制串口状态检测频率，避免CPU占用过高
    // 举例：若USB_PROTECT_SLEEP_TIME=1000ms，则每秒检测一次串口状态
    //      若设为0ms，则while循环会疯狂占用CPU进行无意义的高频检测
    std::this_thread::sleep_for(std::chrono::milliseconds(USB_PROTECT_SLEEP_TIME));
  }
}









/********************************************************/
/* 数据接收处理                                          */
/********************************************************/





void StandardRobotPpRos2Node::receiveData()
{
  RCLCPP_INFO(get_logger(), "Start receiveData with FSM (Standard BCP Protocol)!");

  // 1. 定义解析状态
  enum class RxState {
    SEARCH_SOF,    // 寻找帧起始字节
    READ_HEADER,   // 读取帧头剩余字节 (D_ADDR, ID, LEN)
    READ_DATA,     // 读取载荷 (Data + Checksum)
  };

  // 2. 初始化 FSM 变量（必须在 while 循环外，以处理断帧）
  RxState state = RxState::SEARCH_SOF;
  std::vector<uint8_t> rx_buffer;
  uint8_t payload_len = 0;
  int retry_count = 0;

  // 预分配一次性读取的缓冲区
  std::vector<uint8_t> raw_bytes(256); 

  while (rclcpp::ok()) {
    if (!is_usb_ok_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, 
                           "Receive: USB connection lost! Retry count: %d", retry_count++);
      std::this_thread::sleep_for(std::chrono::milliseconds(USB_NOT_OK_SLEEP_TIME));
      continue;
    }

    try {
      // 3. 从串口读取当前所有可用数据
      int n = serial_driver_->port()->receive(raw_bytes);
      if (n <= 0) continue;

      // 4. 逐字节驱动状态机
      for (int i = 0; i < n; ++i) {
        uint8_t byte = raw_bytes[i];

        switch (state) {
          // 临时调试：确认到底收到了什么
          RCLCPP_INFO(get_logger(), "Raw byte: 0x%02X", byte);
          case RxState::SEARCH_SOF:
            if (byte == SOF_RECEIVE) {
              rx_buffer.clear();
              rx_buffer.push_back(byte);
              state = RxState::READ_HEADER;
            }
            break;

          case RxState::READ_HEADER:
            rx_buffer.push_back(byte);
            // BCP 帧头固定为 4 字节 (SOF, D_ADDR, ID, LEN)
            if (rx_buffer.size() == 4) {
              // 显式读取长度，避免结构体对齐问题
              payload_len = rx_buffer[3];
                        
              // 长度合法性初步检查
              if (payload_len > 128) { 
                RCLCPP_DEBUG(get_logger(), "Header Check: Invalid Len %d, Resetting FSM", payload_len);
                state = RxState::SEARCH_SOF;
                rx_buffer.clear();
              } else {
                state = RxState::READ_DATA;
              }
            }
            break;

          case RxState::READ_DATA:
            rx_buffer.push_back(byte);
            // 目标总长度 = Header(4) + Data(payload_len) + Checksum(2)
            if (rx_buffer.size() == static_cast<size_t>(4 + payload_len + 2)) {
              
              // 5. 完整包收集完毕，执行校验
              if (verify_bcp_checksum(rx_buffer.data(), rx_buffer.size())) {
                
                BCPHeader* header = reinterpret_cast<BCPHeader*>(rx_buffer.data());
                
                // 业务逻辑分发
                switch (header->id) {
                  case ID_GIMBAL: {
                    // rx_buffer 布局: [SOF(0), D_ADDR(1), ID(2), LEN(3), DATA(4...), SC, AC]
                    // 数据段从索引 4 开始读取
                    
                    ReceiveImuData imu_data;
                    
                    // 假设下位机 GimbalPayload 定义为 3 个 int32_t (yaw, pitch, roll)
                    // 偏移量 4: yaw, 偏移量 8: pitch, 偏移量 12: roll
                    // 注意：缩放系数 10000.0f 必须与下位机一致
                    imu_data.data.yaw   = static_cast<float>(readLittleEndian<int32_t>(rx_buffer, 4)) / 10000.0f;
                    imu_data.data.pitch = static_cast<float>(readLittleEndian<int32_t>(rx_buffer, 8)) / 10000.0f;
                    imu_data.data.roll  = static_cast<float>(readLittleEndian<int32_t>(rx_buffer, 12)) / 10000.0f;
                    
                    // 如果还有角速度，继续按偏移量读取 (每增加 4 字节偏移量读取下一个 int32)
                    imu_data.data.yaw_vel   = static_cast<float>(readLittleEndian<int32_t>(rx_buffer, 16)) / 10000.0f;
                    imu_data.data.pitch_vel = static_cast<float>(readLittleEndian<int32_t>(rx_buffer, 20)) / 10000.0f;
                    imu_data.data.roll_vel  = static_cast<float>(readLittleEndian<int32_t>(rx_buffer, 24)) / 10000.0f;

                    publishImuData(imu_data);
                    break;
                  }
                  case ID_ROBOT_HP: {
                    auto hp_data = fromVector<RobotHpPayload>(rx_buffer);
                    publishAllRobotHp(hp_data);
                    break;
                  }
                  case ID_GAME_STATUS: {
                    auto game_data = fromVector<ReceiveGameStatusData>(rx_buffer);
                    publishGameStatus(game_data);
                    break;
                  }
                  case 0x06: { // 哨兵位姿/状态
                    auto frame = fromVector<ReceivePoseFrame>(rx_buffer);
                    publishSentryPose(frame.data.sentry_pose);
                    break;
                  }
                  default:
                    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000, 
                                         "BCP ID 0x%02X recognized but no handler defined", header->id);
                    break;
                }
              } else {
                // 校验失败，输出原始数据用于排查问题
                std::string raw_hex;
                char buf[5];
                for (auto b : rx_buffer) {
                  snprintf(buf, sizeof(buf), "%02X ", b);
                  raw_hex += buf;
                }
                RCLCPP_ERROR(get_logger(), "BCP Checksum Error! ID: 0x%02X, Data: %s", rx_buffer[2], raw_hex.c_str());
              }
              
              // 处理完毕（或校验失败），重置 FSM 状态
              state = RxState::SEARCH_SOF;
              rx_buffer.clear();
              payload_len = 0;
            }
            break;
        }
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "FSM Exception: %s", ex.what());
      is_usb_ok_ = false;
      state = RxState::SEARCH_SOF;
      rx_buffer.clear();
    }
  }
}

















// void StandardRobotPpRos2Node::publishDebugData(ReceiveDebugData & received_debug_data)
// {
//   static rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr debug_pub;
//   for (auto & package : received_debug_data.packages) {
//     // Create a vector to hold the non-zero data
//     std::vector<uint8_t> non_zero_data;
//     for (unsigned char name : package.name) {
//       if (name != 0) {
//         non_zero_data.push_back(name);
//       } else {
//         break;
//       }
//     }
//     // Convert the non-zero data to a string
//     std::string name(non_zero_data.begin(), non_zero_data.end());

//     if (name.empty()) {
//       continue;
//     }

//     if (debug_pub_map_.find(name) == debug_pub_map_.end()) {
//       createNewDebugPublisher(name);
//     }
//     debug_pub = debug_pub_map_.at(name);

//     example_interfaces::msg::Float64 msg;
//     msg.data = package.data;
//     debug_pub->publish(msg);
//   }
// }






void StandardRobotPpRos2Node::publishImuData(ReceiveImuData & imu_data)
{
  sensor_msgs::msg::JointState joint_msg;
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = joint_msg.header.stamp = now();
  imu_msg.header.frame_id = "gimbal_pitch_odom";

  // Convert Euler angles to quaternion
  tf2::Quaternion q;
  q.setRPY(imu_data.data.roll, imu_data.data.pitch, imu_data.data.yaw);
  imu_msg.orientation.x = q.x();
  imu_msg.orientation.y = q.y();
  imu_msg.orientation.z = q.z();
  imu_msg.orientation.w = q.w();


  imu_msg.angular_velocity.x = imu_data.data.roll_vel;
  imu_msg.angular_velocity.y = imu_data.data.pitch_vel;
  imu_msg.angular_velocity.z = imu_data.data.yaw_vel;
  imu_pub_->publish(imu_msg);

  joint_msg.name = {
    "gimbal_pitch_joint",
    "gimbal_yaw_joint",
    "gimbal_pitch_odom_joint",
    "gimbal_yaw_odom_joint",
  };
  joint_msg.position = {
    imu_data.data.pitch,
    imu_data.data.yaw,
    last_gimbal_pitch_odom_joint_,
    last_gimbal_yaw_odom_joint_,
  };
  joint_state_pub_->publish(joint_msg);

  // 打印C板上传的IMU调试信息
  RCLCPP_INFO(
    get_logger(),
    "IMU Data from C-Board - Yaw: %.3f, Pitch: %.3f, Roll: %.3f, "
    "YawVel: %.3f, PitchVel: %.3f, RollVel: %.3f",
    imu_data.data.yaw, imu_data.data.pitch, imu_data.data.roll,
    imu_data.data.yaw_vel, imu_data.data.pitch_vel, imu_data.data.roll_vel);
}










// void StandardRobotPpRos2Node::publishRobotInfo(ReceiveRobotInfoData & robot_info)
// {
//   // 发布机器人状态信息到 "serial/robot_state_info" 话题
//   // RobotStateInfo 消息用于发布机器人的整体状态信息
//   // 包含时间戳、参考坐标系以及机器人各模块的型号信息
//   pb_rm_interfaces::msg::RobotStateInfo msg;

//   // 将毫秒级时间戳转换为 ROS2 的 Time 类型
//   // sec: 秒部分，nanosec: 纳秒部分
//   msg.header.stamp.sec = robot_info.time_stamp / 1000;
//   msg.header.stamp.nanosec = (robot_info.time_stamp % 1000) * 1e6;
//   // 设置参考坐标系为 "odom"（里程计坐标系）
//   msg.header.frame_id = "odom";

//   // 从型号映射表中获取各模块的型号名称
//   // chassis: 底盘型号，如麦轮底盘、全向轮底盘等
//   msg.models.chassis = robot_models_.chassis.at(robot_info.data.type.chassis);
//   // gimbal: 云台型号，如 yaw_pitch 直连云台
//   msg.models.gimbal = robot_models_.gimbal.at(robot_info.data.type.gimbal);
//   // shoot: 发射机构型号，如摩擦轮+拨弹盘
//   msg.models.shoot = robot_models_.shoot.at(robot_info.data.type.shoot);
//   // arm: 机械臂型号，如 mini 机械臂
//   msg.models.arm = robot_models_.arm.at(robot_info.data.type.arm);
//   // custom_controller: 自定义控制器型号
//   msg.models.custom_controller =
//     robot_models_.custom_controller.at(robot_info.data.type.custom_controller);

//   robot_state_info_pub_->publish(msg);
// }







// void StandardRobotPpRos2Node::publishEventData(ReceiveEventData & event_data)
// {
//   pb_rm_interfaces::msg::EventData msg;

//   msg.non_overlapping_supply_zone = event_data.data.non_overlapping_supply_zone;
//   msg.overlapping_supply_zone = event_data.data.overlapping_supply_zone;
//   msg.supply_zone = event_data.data.supply_zone;

//   msg.small_energy = event_data.data.small_energy;
//   msg.big_energy = event_data.data.big_energy;

//   msg.central_highland = event_data.data.central_highland;
//   msg.trapezoidal_highland = event_data.data.trapezoidal_highland;

//   msg.center_gain_zone = event_data.data.center_gain_zone;

//   event_data_pub_->publish(msg);
// }





void StandardRobotPpRos2Node::publishAllRobotHp(const RobotHpPayload & all_robot_hp)
{
  pb_rm_interfaces::msg::GameRobotHP msg;

  // 1. 提取下位机实际发送的 7 号机器人血量
  msg.red_7_robot_hp = all_robot_hp.red_7_hp;
  msg.blue_7_robot_hp = all_robot_hp.blue_7_hp;

  // 2. 将下位机目前未发送的字段初始化为 0
  msg.red_1_robot_hp = 0;
  msg.red_2_robot_hp = 0;
  msg.red_3_robot_hp = 0;
  msg.red_4_robot_hp = 0;
  msg.red_outpost_hp = 0;
  msg.red_base_hp = 0;

  msg.blue_1_robot_hp = 0;
  msg.blue_2_robot_hp = 0;
  msg.blue_3_robot_hp = 0;
  msg.blue_4_robot_hp = 0;
  msg.blue_outpost_hp = 0;
  msg.blue_base_hp = 0;

  // 3. 发布 ROS2 消息
  all_robot_hp_pub_->publish(msg);
}








void StandardRobotPpRos2Node::publishGameStatus(ReceiveGameStatusData & game_status)
{
  pb_rm_interfaces::msg::GameStatus msg;
  msg.game_progress = game_status.data.game_progress;
  msg.stage_remain_time = game_status.data.stage_remain_time;
  game_status_pub_->publish(msg);

  if (record_rosbag_ && game_status.data.game_progress != previous_game_progress_) {
    previous_game_progress_ = game_status.data.game_progress;
    RCLCPP_INFO(get_logger(), "Game progress: %d", game_status.data.game_progress);

    std::string service_name;
    switch (game_status.data.game_progress) {
      case pb_rm_interfaces::msg::GameStatus::COUNT_DOWN:
        service_name = "start_recording";
        break;
      case pb_rm_interfaces::msg::GameStatus::GAME_OVER:
        service_name = "stop_recording";
        break;
      default:
        return;
    }

    if (!callTriggerService(service_name)) {
      RCLCPP_ERROR(get_logger(), "Failed to call service: %s", service_name.c_str());
    }
  }
}






/**
 * @brief 发布哨兵状态/位姿信息
 * @param sentry_state 下位机上传的原始 1 字节状态位
 */
void StandardRobotPpRos2Node::publishSentryPose(uint8_t sentry_state)
{
  // 假设你有一个对应的消息接口，例如 SentryState
  // pb_rm_interfaces::msg::SentryState msg;
  
  // 如果目前没有专门的消息，通常可以并入 RobotStatus 或单独发布
  auto msg = std::make_unique<example_interfaces::msg::UInt8>();
  msg->data = sentry_state;

  // 业务逻辑解析示例（根据裁判系统手册）：
  // bit 0-1: 哨兵架设状态 (0: 未架设, 1: 架设中, 2: 已架设)
  // uint8_t deploy_status = sentry_state & 0x03; 

  sentry_pose_pub_->publish(std::move(msg));
  
  // 调试信息
  RCLCPP_DEBUG(get_logger(), "Sentry Pose State Published: 0x%02X", sentry_state);
}






// void StandardRobotPpRos2Node::publishRobotMotion(ReceiveRobotMotionData & robot_motion)
// {
//   geometry_msgs::msg::Twist msg;

//   msg.linear.x = robot_motion.data.speed_vector.vx;
//   msg.linear.y = robot_motion.data.speed_vector.vy;
//   msg.angular.z = robot_motion.data.speed_vector.wz;

//   robot_motion_pub_->publish(msg);
// }






// void StandardRobotPpRos2Node::publishGroundRobotPosition(
//   ReceiveGroundRobotPosition & ground_robot_position)
// {
//   pb_rm_interfaces::msg::GroundRobotPosition msg;

//   msg.hero_position.x = ground_robot_position.data.hero_x;
//   msg.hero_position.y = ground_robot_position.data.hero_y;

//   msg.engineer_position.x = ground_robot_position.data.engineer_x;
//   msg.engineer_position.y = ground_robot_position.data.engineer_y;

//   msg.standard_3_position.x = ground_robot_position.data.standard_3_x;
//   msg.standard_3_position.y = ground_robot_position.data.standard_3_y;

//   msg.standard_4_position.x = ground_robot_position.data.standard_4_x;
//   msg.standard_4_position.y = ground_robot_position.data.standard_4_y;

//   ground_robot_position_pub_->publish(msg);
// }







// void StandardRobotPpRos2Node::publishRfidStatus(ReceiveRfidStatus & rfid_status)
// {
//   pb_rm_interfaces::msg::RfidStatus msg;

//   msg.base_gain_point = rfid_status.data.base_gain_point;
//   msg.central_highland_gain_point = rfid_status.data.central_highland_gain_point;
//   msg.enemy_central_highland_gain_point = rfid_status.data.enemy_central_highland_gain_point;
//   msg.friendly_trapezoidal_highland_gain_point =
//     rfid_status.data.friendly_trapezoidal_highland_gain_point;
//   msg.enemy_trapezoidal_highland_gain_point =
//     rfid_status.data.enemy_trapezoidal_highland_gain_point;
//   msg.friendly_fly_ramp_front_gain_point = rfid_status.data.friendly_fly_ramp_front_gain_point;
//   msg.friendly_fly_ramp_back_gain_point = rfid_status.data.friendly_fly_ramp_back_gain_point;
//   msg.enemy_fly_ramp_front_gain_point = rfid_status.data.enemy_fly_ramp_front_gain_point;
//   msg.enemy_fly_ramp_back_gain_point = rfid_status.data.enemy_fly_ramp_back_gain_point;
//   msg.friendly_central_highland_lower_gain_point =
//     rfid_status.data.friendly_central_highland_lower_gain_point;
//   msg.friendly_central_highland_upper_gain_point =
//     rfid_status.data.friendly_central_highland_upper_gain_point;
//   msg.enemy_central_highland_lower_gain_point =
//     rfid_status.data.enemy_central_highland_lower_gain_point;
//   msg.enemy_central_highland_upper_gain_point =
//     rfid_status.data.enemy_central_highland_upper_gain_point;
//   msg.friendly_highway_lower_gain_point = rfid_status.data.friendly_highway_lower_gain_point;
//   msg.friendly_highway_upper_gain_point = rfid_status.data.friendly_highway_upper_gain_point;
//   msg.enemy_highway_lower_gain_point = rfid_status.data.enemy_highway_lower_gain_point;
//   msg.enemy_highway_upper_gain_point = rfid_status.data.enemy_highway_upper_gain_point;
//   msg.friendly_fortress_gain_point = rfid_status.data.friendly_fortress_gain_point;
//   msg.friendly_outpost_gain_point = rfid_status.data.friendly_outpost_gain_point;
//   msg.friendly_supply_zone_non_exchange = rfid_status.data.friendly_supply_zone_non_exchange;
//   msg.friendly_supply_zone_exchange = rfid_status.data.friendly_supply_zone_exchange;
//   msg.friendly_big_resource_island = rfid_status.data.friendly_big_resource_island;
//   msg.enemy_big_resource_island = rfid_status.data.enemy_big_resource_island;
//   msg.center_gain_point = rfid_status.data.center_gain_point;

//   rfid_status_pub_->publish(msg);
// }








// void StandardRobotPpRos2Node::publishRobotStatus(ReceiveRobotStatus & robot_status)
// {
//   pb_rm_interfaces::msg::RobotStatus msg;

//   msg.robot_id = robot_status.data.robot_id;
//   msg.robot_level = robot_status.data.robot_level;
//   msg.current_hp = robot_status.data.current_up;
//   msg.maximum_hp = robot_status.data.maximum_hp;
//   msg.shooter_barrel_cooling_value = robot_status.data.shooter_barrel_cooling_value;
//   msg.shooter_barrel_heat_limit = robot_status.data.shooter_barrel_heat_limit;
//   msg.shooter_17mm_1_barrel_heat = robot_status.data.shooter_17mm_1_barrel_heat;
//   msg.robot_pos.position.x = robot_status.data.robot_pos_x;
//   msg.robot_pos.position.y = robot_status.data.robot_pos_y;
//   msg.robot_pos.orientation =
//     tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), robot_status.data.robot_pos_angle));
//   msg.armor_id = robot_status.data.armor_id;
//   msg.hp_deduction_reason = robot_status.data.hp_deduction_reason;
//   msg.projectile_allowance_17mm = robot_status.data.projectile_allowance_17mm;
//   msg.remaining_gold_coin = robot_status.data.remaining_gold_coin;

//   if (last_hp_ - msg.current_hp > 0) {
//     msg.is_hp_deduced = true;
//   }
//   last_hp_ = robot_status.data.current_up;

//   robot_status_pub_->publish(msg);

//   if (set_detector_color_) {
//     uint8_t detect_color;
//     if (getDetectColor(robot_status.data.robot_id, detect_color)) {
//       if (!initial_set_param_ || detect_color != previous_receive_color_) {
//         previous_receive_color_ = detect_color;
//         setParam(rclcpp::Parameter("detect_color", detect_color));
//         std::this_thread::sleep_for(std::chrono::milliseconds(500));
//       }
//     }
//   }
// }





// void StandardRobotPpRos2Node::publishJointState(ReceiveJointState & packet)
// {
//   last_gimbal_pitch_odom_joint_ = packet.data.pitch;
//   last_gimbal_yaw_odom_joint_ = packet.data.yaw;
// }




// void StandardRobotPpRos2Node::publishBuff(ReceiveBuff & buff)
// {
//   pb_rm_interfaces::msg::Buff msg;
//   msg.recovery_buff = buff.data.recovery_buff;
//   msg.cooling_buff = buff.data.cooling_buff;
//   msg.defence_buff = buff.data.defence_buff;
//   msg.vulnerability_buff = buff.data.vulnerability_buff;
//   msg.attack_buff = buff.data.attack_buff;
//   msg.remaining_energy = buff.data.remaining_energy;
//   buff_pub_->publish(msg);
// }














/********************************************************/
/* Send data                                            */
/********************************************************//**
 * @brief 机器人控制指令发送线程
 * 核心整改：摒弃结构体整体拷贝，改用逐字节构建（appendLittleEndian），
 * 确保数据在内存中无空隙（Padding）且符合小端序标准。
 */
void StandardRobotPpRos2Node::sendData()
{
  RCLCPP_INFO(get_logger(), "Start sendData with Precise Endianness Handling!");

  int retry_count = 0;

  while (rclcpp::ok()) {
    // 检查串口状态
    if (!is_usb_ok_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000, 
                           "Send: USB is not ready. Retry count: %d", retry_count++);
      std::this_thread::sleep_for(std::chrono::milliseconds(USB_NOT_OK_SLEEP_TIME));
      continue;
    }

    try {
      // 1. 初始化待发送的字节流容器
      std::vector<uint8_t> frame;
      frame.reserve(32); // 预分配空间提高效率

      // 2. 填充固定帧头 (Header)
      frame.push_back(SOF_SEND);       // 0xFF
      frame.push_back(0x03);           // 目标地址 (D_ADDR)，需与下位机 C 板配置一致
      frame.push_back(ID_ROBOT_CMD);   // 机器人控制指令包 ID

      // 3. 构造载荷 (Payload) 并处理小端序
      // 这里的重点是利用 appendLittleEndian 显式控制每个字节的物理排布
      std::vector<uint8_t> payload;
      
      // vx, vy, wz 缩放系数需与下位机接收逻辑对齐 (通常为 10000.0)
      // appendLittleEndian 确保了 int32_t 的低位字节（LSB）在前，高位字节（MSB）在后
      appendLittleEndian(payload, static_cast<int32_t>(send_robot_cmd_data_.data.vx * 10000.0));
      appendLittleEndian(payload, static_cast<int32_t>(send_robot_cmd_data_.data.vy * 10000.0));
      appendLittleEndian(payload, static_cast<int32_t>(send_robot_cmd_data_.data.wz * 10000.0));
      
      // 4. 填充载荷长度字段
      frame.push_back(static_cast<uint8_t>(payload.size()));

      // 5. 将构造好的载荷插入到完整帧中
      frame.insert(frame.end(), payload.begin(), payload.end());

      // 6. 计算并追加 BCP 校验和 (SC 和 AC)
      // 此时计算范围包含了从 SOF 到 DATA 结束的所有字节
      uint8_t sc = 0;
      uint8_t ac = 0;
      calculate_bcp_checksum(frame.data(), frame.size(), sc, ac);
      frame.push_back(sc);
      frame.push_back(ac);

      // 7. 执行物理发送
      // 只有这种手动构建的 vector 才能保证 100% 不受 CPU 对齐规则的影响
      serial_driver_->port()->send(frame);

    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error in sendData loop: %s", ex.what());
      is_usb_ok_ = false;
    }

    // 控制发送频率 (5ms 即 200Hz)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}







void StandardRobotPpRos2Node::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  send_robot_cmd_data_.data.vx = msg->linear.x;
  send_robot_cmd_data_.data.vy = msg->linear.y;
  send_robot_cmd_data_.data.wz = msg->angular.z;
}






/**
 * @brief 云台关节状态回调函数 - 接收ROS2关节状态消息并更新发送给下位机的云台控制数据
 * 
 * @调用时机: 当订阅的 "cmd_gimbal_joint" 话题发布 sensor_msgs::msg::JointState 消息时自动调用
 *           通常由上层控制节点（如gimbal_controller或teleop节点）以一定频率（如50Hz）发布
 *           用于实时更新云台的期望pitch和yaw角度，通过串口发送给下位机MCU执行
 * 
 * @逻辑说明:
 * 1. 首先检查消息中name数组和position数组长度是否一致，防止数据异常
 * 2. 遍历所有关节名称，查找"gimbal_pitch_joint"和"gimbal_yaw_joint"
 * 3. 将对应的位置值（弧度）赋值给发送数据结构体中的gimbal.pitch和gimbal.yaw字段
 * 4. 这些数据将在sendData()线程中被打包成BCP协议帧，通过串口发送给下位机
 */
// void StandardRobotPpRos2Node::cmdGimbalJointCallback(
//   const sensor_msgs::msg::JointState::SharedPtr msg)
// {
//   // 校验消息数据完整性：关节名称数组与位置数组长度必须一致
//   if (msg->name.size() != msg->position.size()) {
//     RCLCPP_ERROR(
//       get_logger(), "JointState message name and position arrays are of different sizes");
//     return;
//   }

//   // 遍历所有关节，提取云台pitch和yaw的目标角度
//   for (size_t i = 0; i < msg->name.size(); ++i) {
//     if (msg->name[i] == "gimbal_pitch_joint") {
//       // 将pitch关节的期望位置（弧度）存入发送数据结构
//       send_robot_cmd_data_.data.gimbal.pitch = msg->position[i];
//     } else if (msg->name[i] == "gimbal_yaw_joint") {
//       // 将yaw关节的期望位置（弧度）存入发送数据结构
//       send_robot_cmd_data_.data.gimbal.yaw = msg->position[i];
//     }
//   }
//   // 注意：实际发送在独立的sendData()线程中进行，通过串口以固定频率（如200Hz）下发
// }





// void StandardRobotPpRos2Node::visionTargetCallback(
//   const auto_aim_interfaces::msg::Target::SharedPtr msg)
// { 
//   std::lock_guard<std::mutex> lock(send_data_mutex_);
//   send_robot_cmd_data_.data.tracking.tracking = msg->tracking;
// }




// void StandardRobotPpRos2Node::cmdShootCallback(const example_interfaces::msg::UInt8::SharedPtr msg)
// {
//   send_robot_cmd_data_.data.shoot.fric_on = true;
//   send_robot_cmd_data_.data.shoot.fire = msg->data;
// }







void StandardRobotPpRos2Node::setParam(const rclcpp::Parameter & param)
{
  if (!initial_set_param_) {
    auto node_graph = this->get_node_graph_interface();
    auto node_names = node_graph->get_node_names();
    std::vector<std::string> possible_detectors = {
      "armor_detector_openvino", "armor_detector_opencv"};

    for (const auto & name : possible_detectors) {
      for (const auto & node_name : node_names) {
        if (node_name.find(name) != std::string::npos) {
          detector_node_name_ = node_name;
          break;
        }
      }
      if (!detector_node_name_.empty()) {
        break;
      }
    }

    if (detector_node_name_.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000, "No detector node found!");
      return;
    }

    detector_param_client_ =
      std::make_shared<rclcpp::AsyncParametersClient>(this, detector_node_name_);
    if (!detector_param_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), 1000, "Service not ready, skipping parameter set");
      return;
    }
  }

  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
    set_param_future_ = detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
        initial_set_param_ = true;
      });
  }
}








bool StandardRobotPpRos2Node::getDetectColor(uint8_t robot_id, uint8_t & color)
{
  if (robot_id == 0 || (robot_id > 11 && robot_id < 101)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), 1000, "Invalid robot ID: %d. Color not set.", robot_id);
    return false;
  }
  color = (robot_id >= 100) ? 0 : 1;
  return true;
}










bool StandardRobotPpRos2Node::callTriggerService(const std::string & service_name)
{
  auto client = this->create_client<std_srvs::srv::Trigger>(service_name);
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto start_time = std::chrono::steady_clock::now();
  while (!client->wait_for_service(0.1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        get_logger(), "Interrupted while waiting for the service: %s", service_name.c_str());
      return false;
    }
    auto elapsed_time = std::chrono::steady_clock::now() - start_time;
    if (elapsed_time > std::chrono::seconds(5)) {
      RCLCPP_ERROR(
        get_logger(), "Service %s not available after 5 seconds, giving up.", service_name.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "Service %s not available, waiting again...", service_name.c_str());
  }

  auto result = client->async_send_request(request);
  if (
    rclcpp::spin_until_future_complete(this->shared_from_this(), result) ==
    rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(
      get_logger(), "Service %s call succeeded: %s", service_name.c_str(),
      result.get()->success ? "true" : "false");
    return result.get()->success;
  }

  RCLCPP_ERROR(get_logger(), "Service %s call failed", service_name.c_str());
  return false;
}
// ---------------- 复制以下代码，替换文件末尾 ----------------

void StandardRobotPpRos2Node::calculate_SC_AC(uint8_t *ptr, uint8_t len, uint8_t &sc, uint8_t &ac) {
    uint8_t sum = 0, add = 0;
    for(int i = 0; i < len; i++) {
        sum += ptr[i];
        add += sum;
    }
    sc = sum;
    ac = add;
}
// ---------------- 替换开始 ----------------

// 专门为配合下位机 Bug 定制的校验和计算函数
void StandardRobotPpRos2Node::calculate_bcp_checksum(const uint8_t *data, uint32_t len, uint8_t &sc, uint8_t &ac) {
    uint8_t sum = 0;
    uint8_t add = 0;

    // 1. 完美复刻下位机的“偷工减料” Bug：
    // 前 4 个字节 (HEAD, D_ADDR, ID, LEN) 连加 4 次 sum，但 add 只加 1 次
    sum += data[0]; // HEAD
    sum += data[1]; // D_ADDR
    sum += data[2]; // ID
    sum += data[3]; // LEN
    add += sum;     // 就是这里！模仿下位机漏掉中间累加的逻辑

    // 2. 剩下的 DATA 部分，按照正常逻辑逐字节累加
    // data[4] 是 DATA 载荷的第一个字节，一直遍历到 len-1
    for(uint32_t i = 4; i < len; i++) {
        sum += data[i];
        add += sum;
    }
    
    sc = sum;
    ac = add;
}

// 恢复严格校验的验证函数
bool StandardRobotPpRos2Node::verify_bcp_checksum(const uint8_t * data, uint32_t len)
{
  if (data == nullptr || len < 6) {
    return false;
  }
  uint8_t sc_calc = 0;
  uint8_t ac_calc = 0;

  // 调用我们刚刚写好的“带病兼容版”计算函数
  calculate_bcp_checksum(data, len - 2, sc_calc, ac_calc);

  bool sc_match = (data[len - 2] == sc_calc);
  bool ac_match = (data[len - 1] == ac_calc);

  // 既然我们已经配合了下位机的算法，现在就可以理直气壮地要求 SC 和 AC 必须双双匹配了！
  return (sc_match && ac_match);
}

void StandardRobotPpRos2Node::append_bcp_checksum(uint8_t * data, uint32_t len)
{
  if (data == nullptr || len < 6) {
    return;
  }
  uint8_t sc = 0;
  uint8_t ac = 0;

  calculate_bcp_checksum(data, len - 2, sc, ac);

  data[len - 2] = sc;
  data[len - 1] = ac;
}

}  // namespace standard_robot_pp_ros2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(standard_robot_pp_ros2::StandardRobotPpRos2Node)

// ---------------- 替换结束 ----------------