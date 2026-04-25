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

#ifndef STANDARD_ROBOT_PP_ROS2__PACKET_TYPEDEF_HPP_
#define STANDARD_ROBOT_PP_ROS2__PACKET_TYPEDEF_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace standard_robot_pp_ros2
{

const uint8_t SOF_RECEIVE = 0xFF;
const uint8_t SOF_SEND    = 0xFF;

const uint8_t ADDR_BROADCAST   = 0x00;
const uint8_t ADDR_MAINFLOD    = 0x01;
const uint8_t ADDR_GIMBAL      = 0x20;

const uint8_t ID_CHASSIS_CTRL  = 0x12;
const uint8_t ID_POSE_CTRL     = 0x15;
const uint8_t ID_GIMBAL        = 0x20;
const uint8_t ID_GAME_STATUS   = 0x30;
const uint8_t ID_ROBOT_HP      = 0x31;
const uint8_t ID_HEARTBEAT     = 0xF0;

const uint8_t ID_ROBOT_CMD     = 0x12;

const uint8_t DEBUG_PACKAGE_NUM = 10;
const uint8_t DEBUG_PACKAGE_NAME_LEN = 10;

struct HeaderFrame
{
  uint8_t sof;
  uint8_t len;
  uint8_t id;
  uint8_t crc;
} __attribute__((packed));

// 帧头结构体用于解析 BCP 协议帧的前 4 字节
struct BCPHeader {
  uint8_t head;
  uint8_t d_addr;
  uint8_t id;
  uint8_t len;
} __attribute__((packed));



// #pragma pack 是 MSVC/GCC 编译器指令，用于控制结构体内存对齐
// push 保存当前对齐状态，1 表示按 1 字节对齐（无填充），pop 恢复之前状态
// 注意：#pragma pack 不需要写在文件开头，作用域仅限于其包围的代码块





#pragma pack(push, 1)
template<typename T>
struct BCPFrame {
  uint8_t head;
  uint8_t d_addr;
  uint8_t id;
  uint8_t len;
  T data;
  uint8_t sc;
  uint8_t ac;
} __attribute__((packed));



struct GimbalPayload {
  uint8_t mode;
  int32_t yaw;
  int32_t pitch;
  int32_t roll;
} __attribute__((packed));

struct PoseReceivePayload {
  uint8_t sentry_pose;
} __attribute__((packed));

using ReceivePoseFrame = BCPFrame<PoseReceivePayload>;

struct ChassisPayload {
  int32_t linear_x;
  int32_t linear_y;
  int32_t linear_z;
  int32_t angular_x;
  int32_t angular_y;
  int32_t angular_z;
} __attribute__((packed));

struct RobotHpPayload {
  uint8_t reserved1[14];
  uint16_t red_7_hp;
  uint8_t reserved2[14];
  uint16_t blue_7_hp;
} __attribute__((packed));

struct GimbalReceivePayload {
  uint8_t mode;
  int32_t yaw;
  int32_t pitch;
  int32_t roll;
} __attribute__((packed));

using ReceiveGimbalFrame = BCPFrame<GimbalReceivePayload>;



#pragma pack(pop)






// ==========================================
// 具体业务帧定义 (Aliasing)
// ==========================================






// ==========================================
// 5. 数据载荷长度预定义
// ==========================================
const uint8_t FRAME_CTRL_LEN = 24;  // CHASSIS_CTRL 数据段长度
const uint8_t FRAME_RPY_LEN  = 25;  // GIMBAL 数据段长度
const uint8_t FRAME_MAX_LEN  = 36;  // 最大允许长度











/********************************************************/
/* Receive data                                         */
/********************************************************/

// 串口调试数据包
struct ReceiveDebugData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;
  struct
  {
    uint8_t name[DEBUG_PACKAGE_NAME_LEN];
    uint8_t type;
    float data;
  } __attribute__((packed)) packages[DEBUG_PACKAGE_NUM];

  uint16_t checksum;
} __attribute__((packed));

// IMU 数据包
struct ReceiveImuData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct
  {
    float yaw;    // rad
    float pitch;  // rad
    float roll;   // rad

    float yaw_vel;    // rad/s
    float pitch_vel;  // rad/s
    float roll_vel;   // rad/s

    // float x_accel;  // m/s^2
    // float y_accel;  // m/s^2
    // float z_accel;  // m/s^2
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));





// 机器人信息数据包
struct ReceiveRobotInfoData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct
  {
    /// @brief 机器人部位类型 2 bytes
    struct
    {
      uint16_t chassis : 3;
      uint16_t gimbal : 3;
      uint16_t shoot : 3;
      uint16_t arm : 3;
      uint16_t custom_controller : 3;
      uint16_t reserve : 1;
    } __attribute__((packed)) type;

    /// @brief 机器人部位状态 1 byte
    /// @note 0: 错误，1: 正常
    struct
    {
      uint8_t chassis : 1;
      uint8_t gimbal : 1;
      uint8_t shoot : 1;
      uint8_t arm : 1;
      uint8_t custom_controller : 1;
      uint8_t reserve : 3;
    } __attribute__((packed)) state;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));





// 事件数据包
struct ReceiveEventData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct
  {
    uint8_t non_overlapping_supply_zone : 1;
    uint8_t overlapping_supply_zone : 1;
    uint8_t supply_zone : 1;

    uint8_t small_energy : 1;
    uint8_t big_energy : 1;

    uint8_t central_highland : 2;
    uint8_t reserved1 : 1;

    uint8_t trapezoidal_highland : 2;

    uint8_t center_gain_zone : 2;

    uint8_t reserved2 : 4;
  } __attribute__((packed)) data;
  uint16_t crc;
} __attribute__((packed));




// PID调参数据包
struct ReceivePidDebugData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;
  struct
  {
    float fdb;
    float ref;
    float pid_out;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));




// 全场机器人hp信息数据包
struct ReceiveAllRobotHpData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct
  {
    uint16_t red_1_robot_hp;
    uint16_t red_2_robot_hp;
    uint16_t red_3_robot_hp;
    uint16_t red_4_robot_hp;
    uint16_t red_7_robot_hp;
    uint16_t red_outpost_hp;
    uint16_t red_base_hp;

    uint16_t blue_1_robot_hp;
    uint16_t blue_2_robot_hp;
    uint16_t blue_3_robot_hp;
    uint16_t blue_4_robot_hp;
    uint16_t blue_7_robot_hp;
    uint16_t blue_outpost_hp;
    uint16_t blue_base_hp;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));






// 比赛信息数据包
struct ReceiveGameStatusData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct
  {
    uint8_t game_progress;
    uint16_t stage_remain_time;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));






// 机器人运动数据包
struct ReceiveRobotMotionData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct
  {
    struct
    {
      float vx;
      float vy;
      float wz;
    } __attribute__((packed)) speed_vector;
  } __attribute__((packed)) data;
  uint16_t crc;
} __attribute__((packed));

// 地面机器人位置数据包
struct ReceiveGroundRobotPosition
{
  HeaderFrame frame_header;
  uint32_t time_stamp;
  struct
  {
    float hero_x;
    float hero_y;

    float engineer_x;
    float engineer_y;

    float standard_3_x;
    float standard_3_y;

    float standard_4_x;
    float standard_4_y;

    float reserved1;
    float reserved2;
  } __attribute__((packed)) data;
  uint16_t crc;
} __attribute__((packed));

// RFID 状态数据包
struct ReceiveRfidStatus
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct
  {
    uint32_t base_gain_point : 1;
    uint32_t central_highland_gain_point : 1;
    uint32_t enemy_central_highland_gain_point : 1;
    uint32_t friendly_trapezoidal_highland_gain_point : 1;
    uint32_t enemy_trapezoidal_highland_gain_point : 1;
    uint32_t friendly_fly_ramp_front_gain_point : 1;
    uint32_t friendly_fly_ramp_back_gain_point : 1;
    uint32_t enemy_fly_ramp_front_gain_point : 1;
    uint32_t enemy_fly_ramp_back_gain_point : 1;
    uint32_t friendly_central_highland_lower_gain_point : 1;
    uint32_t friendly_central_highland_upper_gain_point : 1;
    uint32_t enemy_central_highland_lower_gain_point : 1;
    uint32_t enemy_central_highland_upper_gain_point : 1;
    uint32_t friendly_highway_lower_gain_point : 1;
    uint32_t friendly_highway_upper_gain_point : 1;
    uint32_t enemy_highway_lower_gain_point : 1;
    uint32_t enemy_highway_upper_gain_point : 1;
    uint32_t friendly_fortress_gain_point : 1;
    uint32_t friendly_outpost_gain_point : 1;
    uint32_t friendly_supply_zone_non_exchange : 1;
    uint32_t friendly_supply_zone_exchange : 1;
    uint32_t friendly_big_resource_island : 1;
    uint32_t enemy_big_resource_island : 1;
    uint32_t center_gain_point : 1;
    uint32_t reserved : 8;
  } __attribute__((packed)) data;
  uint16_t crc;
} __attribute__((packed));

// 机器人状态数据包
struct ReceiveRobotStatus
{
  HeaderFrame frame_header;

  uint32_t time_stamp;

  struct
  {
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_up;
    uint16_t maximum_hp;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;

    uint16_t shooter_17mm_1_barrel_heat;

    float robot_pos_x;
    float robot_pos_y;
    float robot_pos_angle;

    uint8_t armor_id : 4;
    uint8_t hp_deduction_reason : 4;

    uint16_t projectile_allowance_17mm;
    uint16_t remaining_gold_coin;
  } __attribute__((packed)) data;
  uint16_t crc;
} __attribute__((packed));

// 云台状态数据包
struct ReceiveJointState
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct
  {
    float pitch;
    float yaw;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// 机器人增益和底盘能量数据包
struct ReceiveBuff
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct
  {
    uint8_t recovery_buff;
    uint8_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
    uint8_t remaining_energy;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));
/********************************************************/
/* Send data                                            */
/********************************************************/





/**
 * @brief 底盘控制数据载荷 (ID: 0x12)
 * 对应下位机中解析 FRAME_CTRL_LEN (24 字节) 的逻辑
 * 包含：vx, vy, vz, wx, wy, wz
 */
struct ChassisControlPayload {
  int32_t vx;
  int32_t vy;
  int32_t vz;
  int32_t wx;
  int32_t wy;
  int32_t wz;
} __attribute__((packed));

using SendChassisFrame = BCPFrame<ChassisControlPayload>;



// /**
//  * @brief BCP 协议通用帧模板
//  * 统一包含 Header + DATA + CheckSum
//  */
// template<typename T>
// struct BCPFrame {
//   // 帧头 (任务 1 已对齐)
//   uint8_t head;        // 0xFF
//   uint8_t d_addr;      // 目标地址 (1字节)
//   uint8_t id;          // 功能码 (1字节)
//   uint8_t len;         // 载荷长度 (sizeof(T))
  
//   // 载荷段
//   T data;
  
//   // 帧尾 (校验位)
//   uint8_t sc;          // 和校验 (Sum Check)
//   uint8_t ac;          // 附加校验 (Add Check)
// } __attribute__((packed));





// struct SendRobotCmdData
// {
//   HeaderFrame frame_header;

//   uint32_t time_stamp;

//   struct
//   {
//     struct
//     {
//       float vx;
//       float vy;
//       float wz;
//     } __attribute__((packed)) speed_vector;

//     struct
//     {
//       float roll;
//       float pitch;
//       float yaw;
//       float leg_lenth;
//     } __attribute__((packed)) chassis;

//     struct
//     {
//       float pitch;
//       float yaw;
//     } __attribute__((packed)) gimbal;

//     struct
//     {
//       uint8_t fire;
//       uint8_t fric_on;
//     } __attribute__((packed)) shoot;

//     struct
//     {
//       bool tracking;
//     } __attribute__((packed)) tracking;
//   } __attribute__((packed)) data;

//   uint16_t checksum;
// } __attribute__((packed));









/********************************************************/
/* template                                             */
/********************************************************/


template <typename T>
inline T fromVector(const std::vector<uint8_t> & data)
{
  // 1. 安全检查：如果数据量不足以填满一个结构体，拒绝执行
  // 这能有效防止因版本不一致导致的垃圾值读入
  if (data.size() < sizeof(T)) {
      // 抛出警告或记录日志
      return T{}; // 返回一个全零初始化的对象
  }

  T packet;
  // 2. 限制拷贝长度：永远不要拷贝超过 sizeof(T) 的数据
  // 这样即便 data 很大，也不会发生内存溢出
  const uint8_t* src_ptr = data.data();
  std::memcpy(&packet, src_ptr, sizeof(T));

  return packet;
}

template <typename T>
inline std::vector<uint8_t> toVector(const T & data)
{
  std::vector<uint8_t> packet(sizeof(T));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data), reinterpret_cast<const uint8_t *>(&data) + sizeof(T),
    packet.begin());
  return packet;
}

}  // namespace standard_robot_pp_ros2

#endif  // STANDARD_ROBOT_PP_ROS2__PACKET_TYPEDEF_HPP_
