# 代码接入与配置说明

## 1. 这次已经完成的内容

- `BSP/motor.c` 和 `BSP/motor.h` 已经整理成服务函数框架。
- `GM6020` 现在分成两种服务方式：
  - 电流环服务
  - 电压环服务
- `M3508` 已经作为独立服务对象接入，后面如果要接底盘电机，可以沿用同一套框架。
- `LK` 电机已经接入 `motor` 层，当前用于拨盘。
- 电机反馈的接收与分发，不再写死在 `BSP_Init()` 里面。
- 现在是哪条 CAN 上挂了哪些电机、每个电机是什么类型、逻辑 ID 是多少，都在上层任务里决定。
- `Algorithm/PID.c` 和 `Algorithm/PID.h` 里已经补了通用浮点 PID。
- 当前 PID 支持：
  - 输出限幅
  - 积分限幅
  - `P / I / D` 分项记录
- `TASK/Control_Task.c` 已经做成双轴云台控制任务。
- `TASK/Shoot_Task.c` 已经做成发射任务，包含：
  - 双摩擦轮速度环
  - 摩擦轮稳定判定
  - LK 拨盘角度环 + 速度环双环闭环
- `TIM6` 现在只负责提供 `1 kHz` 控制节拍，不在中断里直接发控制。
- 串口接收都已经改成：
  - DMA 循环接收
  - 空闲中断搬运
  - 软件环形缓冲

## 2. 当前默认硬件映射

### 2.1 串口

- `USART3`
  - 用途：超核电子 IMU
  - 波特率：`921600`
  - 刷新率：`1000 Hz`
  - 当前解析协议：`HI91`
- `USART6`
  - 用途：RM 裁判系统图传接收端
  - 当前解析协议：`VT03 / VT13`

### 2.2 云台电机

- `pitch`
  - 电机：`GM6020`
  - 模式：电流环
  - 总线：`CAN1`
  - 逻辑 ID：`1`
- `yaw`
  - 电机：`GM6020`
  - 模式：电流环
  - 总线：`CAN2`
  - 逻辑 ID：`1`

### 2.3 发射机构

- 左摩擦轮
  - 电机：`M3508`
  - 模式：电流环
  - 总线：`CAN2`
  - 逻辑 ID：`1`
- 右摩擦轮
  - 电机：`M3508`
  - 模式：电流环
  - 总线：`CAN2`
  - 逻辑 ID：`2`
- 拨盘
  - 电机：`LK`
  - 模式：角度环 + 速度环双环
  - 总线：`CAN2`
  - 逻辑 ID：`5`

## 3. 模块职责

### 3.1 `BSP/BSP.c`

- 负责底层初始化。
- 启动 `USART3 / USART6` 的 DMA 接收。
- 手动打开串口空闲中断。
- 配置 CAN 过滤器。
- 启动 `CAN1 / CAN2` 和 `FIFO0` 接收中断。
- 启动 `TIM6`。
- 轮询环形缓冲，把数据送到：
  - `IMU_Process()`
  - `Remote_Process()`
- 在 `TIM6` 回调里分发：
  - `Control_Task_Timer1kHzCallback()`
  - `Shoot_Task_Timer1kHzCallback()`

### 3.2 `BSP/motor.c` / `BSP/motor.h`

- 只负责电机服务层。
- 负责注册电机服务对象。
- 负责按 CAN 和标准 ID 分发反馈。
- 负责组控制帧并发送。
- 不负责决定电机挂在哪条 CAN。
- 不负责决定电机属于哪个任务。

### 3.3 `TASK/Control_Task.c`

- 只负责云台控制逻辑。
- 在这里决定：
  - `pitch / yaw` 各自挂什么电机
  - 云台用什么控制模式
  - 遥控通道怎么映射
  - IMU 反馈索引怎么取
  - 机械限位是多少
  - 回中角是多少
  - PID 参数是多少
  - 掉线和保护策略是什么

### 3.4 `TASK/Shoot_Task.c`

- 只负责发射相关逻辑。
- 在这里决定：
  - 摩擦轮挂在哪条 CAN
  - 摩擦轮目标转速是多少
  - 摩擦轮什么时候算“转稳”
  - 拨盘挂在哪条 CAN
  - 拨盘每次步进多少角度
  - 拨盘双环 PID 参数是多少
  - 遥控怎么触发摩擦轮和拨盘

### 3.5 `BSP/IMU.c`

- 只负责解析 IMU 串口数据。

### 3.6 `BSP/Remote.c`

- 只负责解析图传遥控数据。

### 3.7 `Core/Src/stm32f4xx_it.c`

- 负责把 `USART3 / USART6` 的空闲中断接到 `BSP_HandleUartIdle()`。
- 负责把 `CAN1 / CAN2` 的接收中断接到 `Motor_ProcessCanMessage()`。
- 负责把 `TIM6` 中断接到上层任务节拍回调。

### 3.8 `Core/Src/main.c`

- 初始化时调用：
  - `BSP_Init()`
  - `Control_Task_Init()`
  - `Shoot_Task_Init()`
- 主循环里调用：
  - `BSP_Poll()`
  - `Control_Task_Run()`
  - `Shoot_Task_Run()`

## 4. 现在在 CubeMX 里需要确认什么

## 4.1 USART3

- 用途：IMU
- 引脚：
  - `PB10 = USART3_TX`
  - `PB11 = USART3_RX`
- 波特率：`921600`
- 模式：`Asynchronous`
- 收发：`TX/RX`
- DMA：
  - `USART3_RX`
  - `DMA1_Stream1`
  - 方向：`Peripheral To Memory`
  - 模式：`Circular`
  - 数据宽度：`Byte`
- 中断：
  - `USART3 global interrupt` 开启
  - `DMA1_Stream1_IRQn` 开启

## 4.2 USART6

- 用途：图传遥控接收
- 引脚：
  - `PC6 = USART6_TX`
  - `PC7 = USART6_RX`
- 波特率：`921600`
- 模式：`Asynchronous`
- 收发：`TX/RX`
- DMA：
  - `USART6_RX`
  - `DMA2_Stream1`
  - 方向：`Peripheral To Memory`
  - 模式：`Circular`
  - 数据宽度：`Byte`
- 中断：
  - `USART6 global interrupt` 开启
  - `DMA2_Stream1_IRQn` 开启

## 4.3 CAN1

- 当前用途：`pitch GM6020`
- 引脚：
  - `PA11 = CAN1_RX`
  - `PA12 = CAN1_TX`
- 波特率：`1 Mbps`
- 当前参数：
  - `Prescaler = 3`
  - `BS1 = 10TQ`
  - `BS2 = 3TQ`
- 中断：
  - `CAN1_RX0_IRQn` 开启

## 4.4 CAN2

- 当前用途：
  - `yaw GM6020`
  - 左右摩擦轮 `M3508`
  - 拨盘 `LK`
- 引脚：
  - `PB12 = CAN2_RX`
  - `PB13 = CAN2_TX`
- 波特率：`1 Mbps`
- 当前参数：
  - `Prescaler = 3`
  - `BS1 = 10TQ`
  - `BS2 = 3TQ`
- 中断：
  - `CAN2_RX0_IRQn` 开启

## 4.5 TIM6

- 用途：`1 kHz` 控制节拍
- 当前参数：
  - `Prescaler = 83`
  - `Period = 999`
- 中断：
  - `TIM6_DAC_IRQn` 开启

## 4.6 DMA

- 当前必须保留：
  - `USART3_RX -> DMA1_Stream1 -> Circular`
  - `USART6_RX -> DMA2_Stream1 -> Circular`
- 这两个不能关。
- 现在的串口框架依赖 DMA 一直循环接收。

## 4.7 哪些东西不用在 CubeMX 里配

- `UART IDLE` 空闲中断不用在 CubeMX 里单独找开关。
- 这部分已经在代码里手动打开。
- `CAN` 过滤器也不用在 CubeMX 里画。
- 过滤器是在 `BSP/BSP.c` 里通过 `HAL_CAN_ConfigFilter()` 配的。

## 5. `Control_Task` 当前默认逻辑

### 5.1 模式名字

- `RELAX`
  - 含义就是无力
  - 直接给 `0` 电流
- `HOLD`
  - 保持当前目标姿态
- `MANUAL`
  - 遥控输入积分成目标角
- `RECENTER`
  - 两轴都拉回回中角
- `DEBUG`
  - 用于联调
  - 灵敏度更低
  - 输出限幅更小

### 5.2 遥控挡位映射

- `C` 挡：`RELAX`
- `N` 挡：`MANUAL`
- `S` 挡：`RECENTER`
- 右侧自定义键按住：`DEBUG`
- 暂停键按下：强制 `RELAX`

### 5.3 保护逻辑

- 遥控掉线：
  - 自动进入 `HOLD`
- IMU 掉线：
  - 强制进入 `RELAX`
- 任一云台电机掉线：
  - 强制进入 `RELAX`
- `pitch` 超出机械限位：
  - 目标角会被拉回安全范围

### 5.4 反馈来源

- 角度反馈：IMU 欧拉角
- 角速度反馈：IMU 角速度
- 当前没有用 GM6020 自身编码器做云台角度闭环

## 6. `Shoot_Task` 当前默认逻辑

### 6.1 摩擦轮

- 左摩擦轮：`CAN2 ID1 M3508`
- 右摩擦轮：`CAN2 ID2 M3508`
- 默认目标转速：
  - 左：`4500 rpm`
  - 右：`-4500 rpm`
- 默认按住左侧自定义键开启摩擦轮
- 默认松开左侧自定义键停摩擦轮
- 按暂停键时也会停摩擦轮
- 任一摩擦轮掉线时也会停摩擦轮

### 6.2 摩擦轮转稳判定

- 两路摩擦轮都要满足下面条件才算转稳：
  - 当前转速达到目标转速的 `85%`
  - 当前转速误差不大于 `350 rpm`
- 上述条件要连续保持 `200` 个 `1 kHz` 周期
- 也就是大约 `200 ms`
- 只有 `friction_ready = 1` 时，拨盘才允许闭环

### 6.3 拨盘

- 电机：`CAN2 ID5 LK`
- 控制方式：
  - 外环角度 PID
  - 内环速度 PID
- 默认一步转动：
  - `45 deg`
- 默认扳机上升沿记一次步进请求
- 当前步进误差收敛到窗口内后，才继续吃下一次步进请求

### 6.4 拨盘联锁

- 只有同时满足下面条件，拨盘才允许进入闭环：
  - 摩擦轮模式为 `RUN`
  - `friction_ready = 1`
  - LK 电机在线
- 也就是说：
  - 摩擦轮没起稳，拨盘不闭环
  - LK 掉线，拨盘不闭环
  - 按暂停键，拨盘停机

### 6.5 已提供的软件接口

- `Shoot_Task_SetFrictionEnable(enable)`
  - 软件开关摩擦轮
- `Shoot_Task_SetFrictionTargetRpm(left, right)`
  - 软件改摩擦轮目标转速
- `Shoot_Task_RequestDialStep(step_count)`
  - 软件发拨盘步进请求
- `Shoot_Task_GetState()`
  - 读发射任务状态

## 7. 后面要改配置，应该去哪里改

## 7.1 改云台电机挂载

- 文件：`TASK/Control_Task.c`
- 位置：顶部“电机挂载配置区”
- 主要宏：
  - `CONTROL_PITCH_MOTOR_CAN`
  - `CONTROL_PITCH_MOTOR_ID`
  - `CONTROL_YAW_MOTOR_CAN`
  - `CONTROL_YAW_MOTOR_ID`
- 以后换 CAN、换逻辑 ID，只改这里。
- 不要去改 `BSP_Init()`。
- 也不要去改 `motor.c` 的反馈分发逻辑。

## 7.2 改云台模式映射

- 文件：`TASK/Control_Task.c`
- 位置：顶部“模式映射配置区”
- 当前宏：
  - `CONTROL_MODE_GEAR_RELAX = REMOTE_SWITCH_C`
  - `CONTROL_MODE_GEAR_MANUAL = REMOTE_SWITCH_N`
  - `CONTROL_MODE_GEAR_RECENTER = REMOTE_SWITCH_S`

## 7.3 改调试模式

- 文件：`TASK/Control_Task.c`
- 位置：顶部“调试模式配置区”
- 主要宏：
  - `CONTROL_DEBUG_ENABLE_BY_CUSTOM_RIGHT`
  - `CONTROL_DEBUG_RECENTER_BY_CUSTOM_LEFT`
  - `CONTROL_DEBUG_PITCH_REMOTE_SCALE_RATIO`
  - `CONTROL_DEBUG_YAW_REMOTE_SCALE_RATIO`
  - `CONTROL_DEBUG_PITCH_SPEED_LIMIT_DPS`
  - `CONTROL_DEBUG_YAW_SPEED_LIMIT_DPS`
  - `CONTROL_DEBUG_PITCH_CURRENT_LIMIT`
  - `CONTROL_DEBUG_YAW_CURRENT_LIMIT`

## 7.4 改遥控通道和灵敏度

- 文件：`TASK/Control_Task.c`
- 位置：顶部“遥控映射配置区”
- 主要宏：
  - `CONTROL_REMOTE_YAW_CHANNEL_INDEX`
  - `CONTROL_REMOTE_PITCH_CHANNEL_INDEX`
  - `CONTROL_YAW_REMOTE_SCALE_DEG_PER_S`
  - `CONTROL_PITCH_REMOTE_SCALE_DEG_PER_S`
  - `CONTROL_REMOTE_DEADBAND`

## 7.5 改 IMU 索引

- 文件：`TASK/Control_Task.c`
- 位置：顶部“IMU 映射配置区”
- 主要宏：
  - `CONTROL_IMU_PITCH_ANGLE_INDEX`
  - `CONTROL_IMU_YAW_ANGLE_INDEX`
  - `CONTROL_IMU_PITCH_RATE_INDEX`
  - `CONTROL_IMU_YAW_RATE_INDEX`

## 7.6 改云台限位和回中角

- 文件：`TASK/Control_Task.c`
- 位置：顶部“机械限位与回中配置区”
- 主要宏：
  - `CONTROL_PITCH_MIN_DEG`
  - `CONTROL_PITCH_MAX_DEG`
  - `CONTROL_PITCH_CENTER_DEG`
  - `CONTROL_PITCH_PROTECT_MARGIN_DEG`
  - `CONTROL_YAW_MIN_DEG`
  - `CONTROL_YAW_MAX_DEG`
  - `CONTROL_YAW_CENTER_DEG`
  - `CONTROL_YAW_PROTECT_MARGIN_DEG`

## 7.7 改云台双环 PID

- 文件：`TASK/Control_Task.c`
- 位置：
  - “Pitch 轴 PID 配置区”
  - “Yaw 轴 PID 配置区”
- 当前控制关系：
  - 外环输入：目标角 - IMU 角度
  - 外环输出：目标角速度
  - 内环输入：目标角速度 - IMU 角速度
  - 内环输出：目标电流

## 7.8 改摩擦轮配置

- 文件：`TASK/Shoot_Task.c`
- 位置：
  - “摩擦轮电机挂载配置区”
  - “摩擦轮目标转速配置区”
  - “摩擦轮稳定判定配置区”
  - “摩擦轮速度环 PID 配置区”
- 主要宏：
  - `SHOOT_FRICTION_LEFT_CAN`
  - `SHOOT_FRICTION_LEFT_ID`
  - `SHOOT_FRICTION_RIGHT_CAN`
  - `SHOOT_FRICTION_RIGHT_ID`
  - `SHOOT_FRICTION_LEFT_TARGET_RPM`
  - `SHOOT_FRICTION_RIGHT_TARGET_RPM`
  - `SHOOT_FRICTION_RAMP_RPM_PER_S`
  - `SHOOT_FRICTION_READY_RPM_RATIO`
  - `SHOOT_FRICTION_READY_ERROR_RPM`
  - `SHOOT_FRICTION_READY_HOLD_TICKS`

## 7.9 改拨盘配置

- 文件：`TASK/Shoot_Task.c`
- 位置：
  - “LK 拨盘电机挂载配置区”
  - “拨盘步进与状态轮询配置区”
  - “拨盘角度环 PID 配置区”
  - “拨盘速度环 PID 配置区”
- 主要宏：
  - `SHOOT_DIAL_CAN`
  - `SHOOT_DIAL_ID`
  - `SHOOT_DIAL_STEP_DEG`
  - `SHOOT_DIAL_STEP_ACCEPT_ERROR_DEG`
  - `SHOOT_DIAL_STATE_POLL_TICKS`
  - `SHOOT_DIAL_ANGLE_KP / KI / KD`
  - `SHOOT_DIAL_SPEED_KP / KI / KD`

## 7.10 改图传遥控协议

- 文件：`BSP/Remote.c`
- 当前按 `VT03 / VT13` 解析
- 如果后面换了别的图传协议，就改这里

## 7.11 改 IMU 协议

- 文件：`BSP/IMU.c`
- 当前按 `HI91` 解析
- 如果你手里的超核例程输出格式不是 `HI91`，就改这里

## 7.12 改 GM6020 用电流环还是电压环

- 文件：
  - `TASK/Control_Task.c`
  - `BSP/motor.c`
  - `BSP/motor.h`
- 当前 `pitch / yaw` 都是：
  - `Motor_RegisterGm6020CurrentLoop()`
- 如果以后要改成电压环：
  - 上层注册函数要改成 `Motor_RegisterGm6020VoltageLoop()`
  - 上层输出接口也要按电压环方式使用

## 8. `motor` 层现在怎么用

### 8.1 注册电机

- `GM6020` 电流环：
  - `Motor_RegisterGm6020CurrentLoop(&motor, &hcanx, id)`
- `GM6020` 电压环：
  - `Motor_RegisterGm6020VoltageLoop(&motor, &hcanx, id)`
- `M3508`：
  - `Motor_RegisterM3508CurrentLoop(&motor, &hcanx, id)`
- `LK`：
  - `Motor_RegisterLk(&motor, &hcanx, id)`

### 8.2 反馈分发

- `BSP` 中断收到 CAN 帧后，调用 `Motor_ProcessCanMessage()`
- `motor` 层会同时匹配：
  - 当前是哪条 `CAN`
  - 当前 `StdId` 是多少
- 匹配成功后自动更新对应电机对象

### 8.3 为什么不把电机接收写死在初始化里

- 因为同一套底层可以被很多任务复用
- 同一个工程里，电机挂载经常会变
- 如果把“哪条 CAN 上挂了几个什么电机”写死在底层：
  - 上层改任务时会很痛苦
  - 每次换线都要动底层
  - 代码耦合很重
- 现在的结构更清楚：
  - `BSP` 只管把总线跑起来
  - `motor` 只管注册和收发
  - 具体挂载关系交给任务层

## 9. 任务执行顺序

## 9.1 `Control_Task`

- `TIM6` 每 `1 ms` 触发一次中断
- 中断里只做：
  - `Control_Task_Timer1kHzCallback()`
- 主循环里 `Control_Task_Run()` 做下面这些事：
  1. 取 IMU 状态
  2. 取图传遥控状态
  3. 刷新电机在线状态
  4. 根据在线情况和遥控挡位决定模式
  5. 做模式切换初始化
  6. 根据模式更新目标角
  7. 做限位保护
  8. 跑 `pitch / yaw` 双环 PID
  9. 发 CAN 控制帧

## 9.2 `Shoot_Task`

- `TIM6` 每 `1 ms` 触发一次中断
- 中断里只做：
  - `Shoot_Task_Timer1kHzCallback()`
- 主循环里 `Shoot_Task_Run()` 做下面这些事：
  1. 刷新在线状态
  2. 更新摩擦轮模式
  3. 跑摩擦轮速度环
  4. 刷新 `friction_ready`
  5. 更新拨盘模式
  6. 处理扳机步进请求
  7. 跑拨盘双环或停机轮询
  8. 发摩擦轮 CAN 控制帧
  9. 刷新对外状态结构体

## 10. 重新生成 CubeMX 代码后要检查什么

- `.ioc` 重新生成之后，重点检查这些入口有没有被覆盖：
  - `main.c` 里的 `BSP_Init()`
  - `main.c` 里的 `Control_Task_Init()`
  - `main.c` 里的 `Shoot_Task_Init()`
  - 主循环里的 `BSP_Poll()`
  - 主循环里的 `Control_Task_Run()`
  - 主循环里的 `Shoot_Task_Run()`
  - `stm32f4xx_it.c` 里的串口空闲中断处理
  - `stm32f4xx_it.c` 里的 `TIM6` 中断回调
  - `stm32f4xx_it.c` 里的 CAN 接收中断分发

## 11. 当前最常改的文件

- 改云台挂载、模式、限位、回中、PID：
  - `TASK/Control_Task.c`
- 改发射逻辑、摩擦轮、拨盘：
  - `TASK/Shoot_Task.c`
- 改 PID 算法本体：
  - `Algorithm/PID.c`
  - `Algorithm/PID.h`
- 改 IMU 解析：
  - `BSP/IMU.c`
- 改遥控解析：
  - `BSP/Remote.c`
- 改电机服务层：
  - `BSP/motor.c`
  - `BSP/motor.h`
- 改底层初始化：
  - `BSP/BSP.c`

## 12. 当前环境下还没做的事

- 当前环境里没有 `armclang / armcc`
- 所以这边没法直接替你完成编译验证
- 你本地编译时，建议重点看：
  - `MDK-ARM` 工程里是否已经包含：
    - `Algorithm/PID.c`
    - `TASK/Control_Task.c`
    - `TASK/Shoot_Task.c`
  - `.ioc` 重新生成后，中断入口有没有被覆盖
  - 头文件包含路径是否完整
