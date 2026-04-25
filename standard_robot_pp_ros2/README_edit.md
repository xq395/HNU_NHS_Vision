要将 `StandardRobotPpRos2Node`（下位机为 Linux/ROS2 环境）的通信协议修改为适配 `transmission_task.c`（嵌入式环境，基于 RT-Thread 和 BCP 协议），你需要将原有的 **RoboMaster 官方串口帧格式** 替换为 **BCP (Binary Control Protocol) 协议格式**。

以下是为你制定的开发任务表，重点在于对齐协议头、功能码、校验逻辑及数据包结构。

---

## 🛠️ 任务 1：协议头与功能码宏定义对齐
**目标：** 在 ROS2 端（`packet_typedef.hpp` 或相关头文件）定义与 `transmission_task.h` 完全一致的常量。

* [ ] **对齐起始字节 (SOF)：** 将 `SOF_RECEIVE` 和 `SOF_SEND` 统一修改为 `0xFF`。
* [ ] **定义设备地址码：** 引入 `BROADCAST (0x00)`, `MAINFLOD (0x01)`, `GIMBAL (0x20)` 等宏。
* [ ] **定义功能码 (ID)：** * 底盘控制：`CHASSIS_CTRL (0x12)`
    * 云台姿态：`GIMBAL (0x20)`
    * 机器人血量：`ROBOT_HP (0x31)`
    * 心跳包：`HEARTBEAT (0xF0)`

---

## 🛠️ 任务 2：重构数据结构体 (Struct Alignment)
**目标：** `transmission_task` 使用的是 `__attribute__((packed))` 的 BCP 结构。ROS2 端必须构造相同的内存布局。

* [ ] **定义 BCP 通用帧头结构：**
    ```cpp
    struct BCPHeader {
      uint8_t head;   // 0xFF
      uint8_t d_addr; // 目标地址
      uint8_t id;     // 功能码
      uint8_t len;    // DATA 长度
    } __attribute__((packed));
    ```
* [ ] **按功能定义 DATA 段结构：**
    * 参考 `pack_Rpy`：云台数据包含 13 字节（1字节模式 + 3组 float 转 int32 的数据）。
    * 参考 `CHASSIS_CTRL`：底盘数据包含 24 字节（6 个 float 转 int32 的数据）。
* [ ] **定义帧尾：** `SC` (和校验) 和 `AC` (附加校验)。

---

## 🛠️ 任务 3：更换校验逻辑 (CheckSum Migration)
**目标：** 舍弃原有的 `CRC8/CRC16`，改为 `transmission_task.c` 中的 `SC/AC` 校验。

* [ ] **在 `crc8_crc16.cpp` 类似位置实现 SC/AC 算法：**
    ```cpp
    // 逻辑：sum = head + d_addr + id + len + data[0...n]; add = sum1 + sum2 + ...
    void calculate_SC_AC(uint8_t *ptr, uint8_t len, uint8_t &sc, uint8_t &ac) {
        uint8_t sum = 0, add = 0;
        for(int i = 0; i < len; i++) {
            sum += ptr[i];
            add += sum;
        }
        sc = sum;
        ac = add;
    }
    ```
* [ ] **修改校验函数：** 将 `verify_CRC16_check_sum` 替换为对包尾最后两个字节的 `SC/AC` 验证。



根据 **任务三** 的要求，我们需要彻底弃用原有的 CRC8/CRC16 校验逻辑，并在 ROS2 端实现与下位机 `transmission_task.c` 完全一致的 **SC (Sum Check, 和校验)** 和 **AC (Add Check, 附加校验)** 算法。

以下是完成任务三的任务表：

---


## 🛠️ 任务三：更换校验逻辑 (CheckSum Migration)

### 1. 修改校验工具库 (`crc8_crc16.cpp` & `.hpp`)
由于校验方式已变，建议将文件名或命名空间语义化（例如改为 `checksum.cpp` 或在原有文件中添加新命名空间 `bcp_checksum`），以匹配 BCP 协议。

* [ ] **实现 SC/AC 计算核心函数：**
    根据下位机 `Check_Rpy` 函数逻辑，校验范围包含 `HEAD, D_ADDR, ID, LEN` 以及所有的 `DATA` 字节。
    ```cpp
    // 逻辑：sum = byte1 + byte2 + ...; add = sum1 + sum2 + ...
    void calculate_bcp_checksum(const uint8_t * data, uint32_t len, uint8_t & sc, uint8_t & ac)
    {
      uint8_t sum = 0;
      uint8_t add = 0;
      for (uint32_t i = 0; i < len; ++i) {
        sum += data[i];
        add += sum;
      }
      sc = sum;
      ac = add;
    }
    ```
* [ ] **实现 BCP 校验验证函数：**
    验证收到的最后两个字节是否等于计算出的 SC 和 AC。
* [ ] **实现 BCP 校验添加函数：**
    在发送缓冲区末尾添加计算出的 SC 和 AC。

---

### 2. 重构 `receiveData()` 解析流程
BCP 协议是**变长帧**，且校验位固定为 2 字节。你需要根据任务二定义的结构体调整读取逻辑。

* [ ] **调整数据段读取长度：**
    读取完 4 字节 Header 后，读取长度应为 `header_frame.len + 2`（载荷长度 + SC + AC）。
* [ ] **替换校验调用：**
    删除 `verify_CRC8_check_sum`（因为 BCP 帧头没有独立校验）。
    删除 `verify_CRC16_check_sum`，改为调用 `verify_bcp_checksum`。
* [ ] **校验范围对齐：**
    确保校验计算涵盖了从 `SOF` 到 `DATA` 结束的所有字节。

---

### 3. 更新发送校验逻辑 (`sendData`)
* [ ] **修改 `sendData` 中的校验填充：**
    在调用 `serial_driver_->port()->send()` 之前，将 `crc16::append_CRC16_check_sum` 替换为新实现的 `append_bcp_checksum`。

---

### 4. 代码清理与兼容性检查
* [ ] **移除冗余代码：**
    清理不再使用的 `CRC8_TABLE` 和 `W_CRC_TABLE` 查找表，以减小二进制体积。
* [ ] **日志对齐：**
    将 `RCLCPP_ERROR` 中的 "CRC16 error" 修改为 "BCP Checksum error"，方便调试。

---

### 修改建议（核心算法实现）

在 `standard_robot_pp_ros2` 的工具类中添加如下代码：

```cpp
/**
 * @brief  BCP校验逻辑验证
 * @param  data: 包含帧头、数据和校验位的完整包
 * @param  len: 完整包的总长度
 */
bool verify_bcp_checksum(const uint8_t * data, uint32_t len)
{
  if (len < 6) return false; // Head(4) + CheckSum(2)
  
  uint8_t sc_calc = 0, ac_calc = 0;
  // 校验范围：从 data[0] 到 data[len-3]
  calculate_bcp_checksum(data, len - 2, sc_calc, ac_calc);
  
  return (data[len - 2] == sc_calc && data[len - 1] == ac_calc);
}

/**
 * @brief  添加BCP校验位到包末尾
 */
void append_bcp_checksum(uint8_t * data, uint32_t len)
{
  uint8_t sc = 0, ac = 0;
  calculate_bcp_checksum(data, len - 2, sc, ac);
  data[len - 2] = sc;
  data[len - 1] = ac;
}
```































---

## 🛠️ 任务 4：修改数据发送逻辑 (`sendData`)
**目标：** 将 ROS2 的控制指令（`cmd_vel`, `cmd_gimbal`）打包成 BCP 格式发往 USB 虚拟串口。

* [ ] **修改 `send_robot_cmd_data_` 打包过程：**
    * 底盘：将 `Twist` 里的 `vx, vy, wz` 乘以 `10000.0` 转为 `int32_t` 放入 DATA 段。
    * 云台：将弧度值乘以 `1000.0` 转为 `int32_t` 放入 DATA 段。
* [ ] **更新校验：** 发送前调用 `calculate_SC_AC` 填充最后两个字节。

根据 **任务 4** 的目标，我们需要将 ROS2 原有的 RoboMaster 官方控制包格式彻底迁移为 **BCP 协议格式**。这意味着发送逻辑不再是简单的 CRC 校验，而是涉及数据比例放大、地址定向以及 SC/AC 校验计算。

以下是完成任务 4 的详细重构任务表：

---

## 🛠️ 任务 4：数据发送逻辑重构任务表

### 1. 修改 `sendData` 核心循环 (`standard_robot_pp_ros2.cpp`)
* [ ] **弃用旧结构体**：移除原本基于 `SendRobotCmdData` 的发送逻辑。
* [ ] **实现双通道发送**：由于 BCP 协议根据 `D_ADDR` 定向，需要分别构造底盘包（`ADDR_MAINFLOD/0x01`）和云台包（`ADDR_GIMBAL/0x20`）。
* [ ] **更新校验填充**：将 `crc16::append_CRC16_check_sum` 替换为 `append_bcp_checksum`。

### 2. 重写回调函数中的数据打包逻辑
* [ ] **底盘速度处理 (`cmdVelCallback`)**：
    * 比例放大：将 `linear.x`, `linear.y`, `angular.z` 乘以 `10000.0`。
    * 类型转换：将结果强转为 `int32_t` 存入 `ChassisPayload` 结构体。
* [ ] **云台关节处理 (`cmdGimbalJointCallback`)**：
    * 比例放大：将 `pitch`, `yaw` 弧度值乘以 `1000.0`。
    * 类型转换：强转为 `int32_t` 存入 `GimbalPayload`。
* [ ] **控制模式对齐**：确保 `GimbalPayload.mode` 的定义与下位机控制逻辑一致（如：绝对/相对角度切换）。

### 3. 时间同步与心跳发送 (可选但推荐)
* [ ] **定时发送心跳包**：在 `sendData` 线程中以固定频率（如 1Hz）发送 `ID_HEARTBEAT` 包，以维持下位机 `heart_dt` 的更新。

---

## 📝 发送逻辑重构代码参考

### 1. 结构体初始化与打包示例
在 `standard_robot_pp_ros2.cpp` 的 `sendData` 中，应按如下逻辑操作：

```cpp
void StandardRobotPpRos2Node::sendData() {
  while (rclcpp::ok()) {
    if (is_usb_ok_) {
      // 1. 打包底盘命令 (ID_CHASSIS_CTRL / 0x12)
      SendChassisFrame chassis_frame;
      chassis_frame.head   = SOF_SEND;      // 0xFF
      chassis_frame.d_addr = ADDR_MAINFLOD; // 0x01
      chassis_frame.id     = ID_CHASSIS_CTRL;
      chassis_frame.len    = sizeof(ChassisPayload);
      
      // 数据来源于 cmdVelCallback 填充的缓存
      chassis_frame.data = latest_chassis_cmd_; 

      // 2. 添加 BCP 校验 (SC/AC)
      append_bcp_checksum(reinterpret_cast<uint8_t *>(&chassis_frame), sizeof(SendChassisFrame));

      // 3. 串口发送
      serial_driver_->port()->send(toVector(chassis_frame));
    }
    std::this_thread::sleep_for(5ms);
  }
}
```

### 2. 回调函数中的数据缩放
```cpp
void StandardRobotPpRos2Node::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  // 严格执行任务四要求的 10000.0 倍率放大
  latest_chassis_cmd_.linear_x  = static_cast<int32_t>(msg->linear.x * 10000.0);
  latest_chassis_cmd_.linear_y  = static_cast<int32_t>(msg->linear.y * 10000.0);
  latest_chassis_cmd_.angular_z = static_cast<int32_t>(msg->angular.z * 10000.0);
}

void StandardRobotPpRos2Node::cmdGimbalJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  // 严格执行任务四要求的 1000.0 倍率放大
  // 遍历 msg->name 找到 pitch 和 yaw 对应的 index
  // latest_gimbal_cmd_.pitch = static_cast<int32_t>(pitch_val * 1000.0);
  // latest_gimbal_cmd_.yaw   = static_cast<int32_t>(yaw_val * 1000.0);
}
```

---

### ⚠️ 关键落地检查点
1. **数据溢出**：确保 `msg->linear.x * 10000.0` 的结果不会超过 `int32_t` 的范围（通常底盘速度不会超过 214,748 m/s，所以是安全的）。
2. **符号一致性**：下位机 `transmission_task.c` 中 `trans_fdb.yaw` 带有负号映射，请务必根据实际物理方向在 ROS2 端或下位机端统一极性。
3. **发送频率**：BCP 协议较轻量，保持 5ms-10ms 的发送周期即可满足控制实时性。

**任务四完成后，整个通信闭环将建立。是否需要我为您整合一个完整的、基于 BCP 协议的 `sendData` 函数？**




为了完成任务 4 中发送逻辑的重构，我们需要将数据从 ROS2 的消息格式（浮点数、SI 单位）转换为 BCP 协议要求的整数比例格式。

以下是实现 **`cmdVelCallback`** 和 **`cmdGimbalJointCallback`** 数据缩放与打包的详细任务表：

---

### 🛠️ 任务 4.2：回调函数数据缩放与缓存落地

#### 1. 定义数据缓存 (在类成员变量中)
* [ ] 在 `StandardRobotPpRos2Node` 类定义中，增加用于缓存最新指令的载荷结构体，避免在回调函数中频繁分配内存。
    ```cpp
    // 缓存最新的载荷数据，供 sendData() 线程读取
    ChassisPayload latest_chassis_cmd_;
    GimbalPayload latest_gimbal_cmd_;
    std::mutex cmd_mutex_; // 保证线程安全
    ```

#### 2. 实现底盘缩放 (`cmdVelCallback`)
* [ ] **单位转换与放大**：将 `linear.x/y` (m/s) 和 `angular.z` (rad/s) 乘以 **10000.0**。
* [ ] **整数强制转换**：将结果转换为 `int32_t`。
* [ ] **线程安全写入**：使用 `std::lock_guard` 写入缓存。
    ```cpp
    void StandardRobotPpRos2Node::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      latest_chassis_cmd_.linear_x  = static_cast<int32_t>(msg->linear.x * 10000.0);
      latest_chassis_cmd_.linear_y  = static_cast<int32_t>(msg->linear.y * 10000.0);
      latest_chassis_cmd_.angular_z = static_cast<int32_t>(msg->angular.z * 10000.0);
    }
    ```

#### 3. 实现云台缩放 (`cmdGimbalJointCallback`)
* [ ] **关节映射**：遍历 `JointState` 的 `name` 数组，正确识别 `gimbal_pitch_joint` 和 `gimbal_yaw_joint`。
* [ ] **比例放大**：将弧度值乘以 **1000.0**。
* [ ] **方向对齐**：根据下位机 `transmission_task.c` 的解析逻辑（如 `trans_fdb.yaw = -(*int32 / 1000.0)`），在发送端对齐符号极性。
    ```cpp
    void StandardRobotPpRos2Node::cmdGimbalJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "gimbal_pitch_joint") {
          latest_gimbal_cmd_.pitch = static_cast<int32_t>(msg->position[i] * 1000.0);
        } else if (msg->name[i] == "gimbal_yaw_joint") {
          // 对齐下位机逻辑，注意符号变换
          latest_gimbal_cmd_.yaw = static_cast<int32_t>(msg->position[i] * 1000.0);
        }
      }
      latest_gimbal_cmd_.mode = 0; // 默认绝对角度模式
    }
    ```

#### 4. 集成到 `sendData`
* [ ] 修改 `sendData` 线程，从 `latest_chassis_cmd_` 和 `latest_gimbal_cmd_` 读取数据并进行 `append_bcp_checksum` 操作。

---

### 💡 核心逻辑检查点
1. **精度对齐**：
    * **底盘**：下位机 `/ 10000.0`，发送端必须 `* 10000.0`。
    * **云台**：下位机 `/ 1000.0`，发送端必须 `* 1000.0`。
2. **极性一致性**：下位机解析 Yaw 时带有负号，如果 ROS2 侧已经按正确坐标系给出，请确认此处是否需要二次取反。
3. **数据频率**：`JointState` 发布通常较快，确保 `cmd_mutex_` 不会造成 `sendData` 线程的长时间阻塞。

通过这个表，你可以逐步将 ROS2 的控制量完美“翻译”给下位机解析。准备好将这些逻辑写进代码了吗？



解决任务四时候发现发送逻辑出现了大的问题，我们是采用变长协议来传输的，但是pb采用的是一个很长的包来把所有数据同意发送给下位机，这样就会出问题，
针对这个问题，我们列出个小的任务表，来重构发送的数据，和发送逻辑：

发现这个问题非常关键。在串口通信中，如果发送方（ROS2）发送的字节长度超过了接收方（下位机）针对特定 `ID` 预期的长度，会导致下位机缓冲区逻辑错误，从而引发数据解析移位或后续包被丢弃。

由于下位机 `transmission_task.c` 是根据不同的 `ID`（如 `0x12` 为底盘，`0x20` 为云台）来决定读取长度的，你必须将原本“全家桶”式的 `SendRobotCmdData` 拆分为**离散的、功能单一的 BCP 帧**。

以下是针对该问题的重构任务表：

---

## 🛠️ 任务 4.3：发送数据结构与逻辑重构表

### 1. 拆分数据结构 (Structure Decoupling)
* [ ] **弃用全量结构体**：停止使用包含所有模块（底盘、云台、射击、追踪）的 `SendRobotCmdData` 结构体。
* [ ] **建立功能载荷映射**：在 `packet_typedef.hpp` 中确保 `ChassisPayload` (24字节) 和 `GimbalPayload` (13字节) 等载荷与下位机预期的载荷长度完全一致。
* [ ] **引入 BCP 帧模板**：利用之前定义的 `BCPFrame<T>` 模板，确保每一帧都由 `Head(4) + Payload(N) + Checksum(2)` 组成，实现物理上的变长传输。

### 2. 重构回调缓存逻辑 (Callback Buffering)
* [ ] **独立缓存化**：在类成员中为每个模块设置独立的指令缓存（如 `latest_chassis_cmd_`），不再共用一个大结构体。
* [ ] **状态标志位**：为每个缓存增加一个“更新标志” (Dirty Flag) 或时间戳，用于判断是否有新数据需要发送。

### 3. 重写发送执行逻辑 (Execution Logic)
* [ ] **顺序/分包发送**：修改 `sendData()` 线程。在每一个循环周期内，根据标志位依次发送不同的 BCP 包。
    * *例如：先打包发送底盘包，紧接着打包发送云台包。*
* [ ] **动态长度序列化**：调用 `toVector()` 时，针对具体的 `BCPFrame<T>` 进行转换，确保发出的 `std::vector` 长度与该 ID 对应的协议长度严格相等。

### 4. 比例缩放与类型校准 (Scaling & Typing)
* [ ] **强制类型检查**：确保所有 `float` 在打包前都已按照下位机要求（1000/10000倍）转换为 `int32_t`，防止内存布局错误。
* [ ] **符号位对齐**：核对下位机 `transmission_task.c` 中的正负号处理（如 Yaw 轴），确保控制极性正确。

---

## 💡 逻辑对比：重构前 vs 重构后



| 特性 | 重构前 (PB 逻辑) | 重构后 (BCP 变长逻辑) |
| :--- | :--- | :--- |
| **发送方式** | 一个固定的大包（约 60 字节） | 多个针对性的短包（如 30字节/19字节） |
| **下位机解析** | 无法处理冗余字节，导致缓冲区错位 | 根据 `ID` 精确读取 `LEN` 长度，解析稳定 |
| **数据类型** | 直接发送 `float` (IEEE 754) | 发送缩放后的 `int32_t` 定点数 |
| **校验范围** | 对整个大包进行单次校验 | 每帧独立进行 SC/AC 校验 |

### 下一步行动
建议你先从 `StandardRobotPpRos2Node` 类定义中删除旧的 `send_robot_cmd_data_` 成员，并替换为独立的 `latest_chassis_frame_` 等。这样编译器会通过报错指引你修改所有不兼容的旧代码。






重构的话我发现还有一个问题，发送的时候，因为发送任务是并行的，那么这样不会导致串口发送的时候数据相互干扰吗，这样的问题改如何解决

你的观察非常敏锐。在回答如何解决之前，我们先根据你提供的 `standard_robot_pp_ros2.cpp` 源码分析一下：**在当前的代码架构下，是否真的会出现串口发送数据干扰的问题？**

---

## 🔍 现状分析：你的代码会出问题吗？

**结论：在当前的单线程配置下暂时不会，但存在严重的架构隐患。**

### 1. 为什么“现在”可能没出问题？
在 `standard_robot_pp_ros2.cpp` 中，你使用了 ROS2 的默认执行器（`SingleThreadedExecutor`）。
* **串行回调**：所有的订阅者回调函数（`cmdVelCallback`、`cmdGimbalJointCallback` 等）都在同一个线程中按顺序执行。一个回调没跑完，另一个就不会开始。
* **发送线程独立**：你开启了一个专门的 `send_thread_` 来执行 `sendData` 循环。
* **目前的逻辑缺陷**：目前的 `sendData` 是**唯一**调用 `serial_driver_->port()->send()` 的地方。只要你没有在回调函数里直接调用 `send`，就不会发生两个线程同时抢占硬件串口导致的“字节交织”。

### 2. 真正的风险点在哪里？
虽然物理上的串口抢占没发生，但**数据破坏（Data Corruption）**已经发生了：
* **异步读写冲突**：`sendData` 线程在循环读取 `send_robot_cmd_data_` 的字节，而回调函数（如 `cmdVelCallback`）在另一个线程里修改这个结构体的成员。
* **非原子性操作**：修改一个 `float` 或 `int32_t` 需要多个 CPU 周期。可能出现发送线程刚读完 `vx` 的前 2 个字节，回调函数就修改了后 2 个字节，导致发出的数值是一个“半新半旧”的非法巨量数值。

---

## 🛠️ 重构任务表：解决并发干扰与数据安全

为了适配变长协议并彻底消除隐患，建议采用**“单点发送 + 影子缓存”**的重构策略。

### 第一阶段：建立影子缓存（保护数据安全）
不再直接修改全局的大结构体，而是为每个业务模块建立独立的“影子变量”。

* [ ] **定义私有成员变量**：在类中添加底盘载荷 `latest_chassis_payload_` 和云台载荷 `latest_gimbal_payload_`。
* [ ] **引入互斥锁**：添加 `std::mutex cmd_mutex_`。
* [ ] **重写回调函数**：
    * 在回调函数中使用 `std::lock_guard<std::mutex> lock(cmd_mutex_)`。
    * 将缩放后的数据（如 `vx * 10000.0`）存入对应的影子载荷中。

### 第二阶段：重构 `sendData` 执行流（解决干扰与变长包）
将原本“发一个大包”改为“轮询影子变量，按需发多个小包”。

* [ ] **数据提取**：在 `sendData` 循环起始处，加锁并拷贝出影子载荷副本，然后立即解锁。
* [ ] **底盘包发送**：
    * 构造 `SendChassisFrame`。
    * 填充 `D_ADDR = 0x01`，`ID = 0x12`。
    * 调用 `append_bcp_checksum` 并通过串口发出。
* [ ] **云台包发送**：
    * 紧接着构造 `SendGimbalFrame`。
    * 填充 `D_ADDR = 0x20`，`ID = 0x20`。
    * 调用校验并发出。

### 第三阶段：确保发送原子性（防止硬件冲突）
虽然目前只有 `sendData` 在发，但为了防止未来其他功能（如心跳、日志）也想用串口，必须封装原子发送接口。

* [ ] **封装 `safeSend` 接口**：
    ```cpp
    void StandardRobotPpRos2Node::safeSerialSend(const std::vector<uint8_t> & data) {
        static std::mutex send_io_mutex; // 静态锁保护底层硬件
        std::lock_guard<std::mutex> lock(send_io_mutex);
        serial_driver_->port()->send(data);
    }
    ```

---

## 💡 为什么这样改？

| 措施 | 解决的问题 |
| :--- | :--- |
| **影子缓存 + Mutex** | 解决“发送线程读”与“回调线程写”的冲突，防止数值撕裂。 |
| **单点发送逻辑** | 物理上保证串口在同一时间只处理一个业务包，符合 BCP 变长协议要求。 |
| **独立分包** | 彻底解决“全家桶大包”导致的下位机缓冲区解析偏移问题。 |

**你是否准备好按照这个逻辑，先从定义 `std::mutex` 和影子缓存结构体开始修改代码？**






---

## 🛠️ 任务 5：修改数据接收逻辑 (`receiveData`)
**目标：** 按照 BCP 协议的变长特性解析数据包。

* [ ] **重写状态机解析：**
    1.  寻找 `0xFF` (HEAD)。
    2.  读取接下来的 3 个字节（`D_ADDR`, `ID`, `LEN`）。
    3.  根据 `LEN` 读取对应长度的 `DATA`。
    4.  读取最后 2 个字节（`SC`, `AC`）。
* [ ] **实现数据转换：**
    * 例如收到 `ID == GIMBAL`：读取 DATA 里的 `int32_t`，除以 `1000.0` 还原为 `float` 弧度，再发布到 ROS2 的 `JointState`。
* [ ] **心跳维护：** 收到 `ID == HEARTBEAT` 时，更新本地时间戳，确保通信链路未断开。

---

## 🛠️ 任务 6：适配虚拟串口特性
**目标：** 针对 USB 虚拟串口 (VCOM) 的不稳定性进行应用层加固。

* [ ] **频率对齐：** `transmission_task` 的云台数据是 1000Hz，血量是 10Hz。ROS2 接收端应增加对应的缓存处理或降频发布，避免阻塞。
* [ ] **异常处理：** USB 虚拟串口在断电重启时，串口号可能会漂移或挂死。在 `serialPortProtect` 中，确保对 `0xFF` 帧头的识别能快速重连同步。

---

### ⚠️ 注意事项：
1.  **大端小端：** STM32 和 Linux (x86/ARM) 通常都是小端模式，直接 `memcpy` 结构体通常可行，但 `transmission_task.c` 中使用了位移操作（如 `*temp_data1 >> 24`），请务必通过打印测试确保 float 还原后的数值正确。
2.  **数据比例：** 注意 `1000.0` (云台) 和 `10000.0` (底盘) 的系数差异，这在下位机代码中是不统一的，必须严格对齐。s