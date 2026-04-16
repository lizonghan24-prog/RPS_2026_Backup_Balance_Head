# 从 `main` 开始的程序运行流程说明

## 1. 先看总入口

程序入口在 `Core/Src/main.c` 里的 `main()`。

当前这份工程从 `main()` 开始，整体顺序是：

1. `HAL_Init()`
2. `SystemClock_Config()`
3. `MX_..._Init()` 初始化 CubeMX 生成的外设句柄
4. `BSP_Init()` 做底层软件初始化
5. `Control_Task_Init()` 初始化云台任务
6. `Shoot_Task_Init()` 初始化发射任务
7. 进入 `while(1)` 主循环

对应代码位置：

- `main()` 入口：`Core/Src/main.c`
- 关键初始化：`Core/Src/main.c` 第 `102 ~ 132` 行附近
- 主循环：`Core/Src/main.c` 第 `138 ~ 147` 行附近

## 2. `main()` 初始化阶段到底做了什么

### 2.1 `HAL_Init()`

这一步是 HAL 的标准初始化。

它主要做：

- 复位外设状态
- 初始化 Flash 接口
- 初始化 `SysTick`

这一步之后，HAL 的时间基准和中断基础环境才算起来。

### 2.2 `SystemClock_Config()`

这一步配置系统时钟。

当前工程里：

- 外部晶振 `HSE`
- 主频走 PLL
- `APB1 = HCLK / 4`
- `APB2 = HCLK / 2`

这会直接影响：

- `TIM6` 的分频计算
- UART 波特率
- CAN 时序

### 2.3 `MX_..._Init()`

这一段是 CubeMX 生成的外设初始化。

当前 `main.c` 里初始化了：

- GPIO
- DMA
- CAN1
- CAN2
- UART4
- UART5
- USART1
- USART2
- USART3
- USART6
- TIM6

注意：

- 到这里为止，只是把硬件句柄和寄存器配置好
- 还没有真正把你的任务跑起来
- 串口 DMA、CAN 过滤器、1 kHz 控制节拍真正启动，是在 `BSP_Init()` 里做的

## 3. `BSP_Init()` 干了什么

`BSP_Init()` 在 `BSP/BSP.c` 里，当前是整个底层软件入口。

对应代码位置：

- `BSP_Init()`：`BSP/BSP.c` 第 `231` 行

它主要做 4 件事。

### 3.1 初始化协议层和电机服务层

先调用：

- `Remote_Init()`
- `IMU_Init()`
- `Motor_Init()`

意思是：

- 图传遥控解析状态先清零
- IMU 解析状态先清零
- 电机注册表先清零

### 3.2 配置两路串口 DMA 接收链路

当前约定是：

- `USART3` 给 IMU
- `USART6` 给图传遥控

这里会做：

1. 建立软件环形缓冲
2. 记录 DMA 缓冲地址
3. 启动 `HAL_UART_Receive_DMA()`
4. 关闭 DMA 半传输和传输完成中断
5. 打开 UART 空闲中断

所以现在串口接收是这条链：

`UART DMA 循环接收 -> 空闲中断触发 -> 搬到软件环形缓冲 -> 主循环里解析`

### 3.3 启动 CAN

`BSP_Init()` 里会分别对 `CAN1` 和 `CAN2`：

1. 配过滤器
2. `HAL_CAN_Start()`
3. 打开 `FIFO0` 接收中断

当前过滤器策略是：

- 先全放开
- 具体是哪种电机、哪个 ID，由 `motor` 层按软件注册表分发

### 3.4 启动 `TIM6`

`TIM6` 当前只做一件事：

- 每 `1 ms` 产生一次中断

但是它不在中断里直接跑控制算法，只是“发节拍”。

这点很重要，因为现在你的控制结构是：

- `TIM6` 中断负责置位节拍
- 主循环负责真正跑任务

## 4. `Control_Task_Init()` 干了什么

位置：

- `TASK/Control_Task.c` 第 `560` 行

它会做这些事：

1. 清空 `gimbal_control` 整个任务状态
2. 默认模式设成 `RELAX`
3. 配置 `pitch` 轴
4. 配置 `yaw` 轴

### 4.1 `pitch` 轴初始化内容

当前配置是：

- `CAN1`
- `GM6020`
- `ID1`
- 电流环模式

同时会初始化：

- 角度环 PID
- 角速度环 PID
- 遥控映射
- IMU 索引
- 限位参数
- 回中角

### 4.2 `yaw` 轴初始化内容

当前配置是：

- `CAN2`
- `GM6020`
- `ID1`
- 电流环模式

同时也会初始化对应的：

- 双环 PID
- 遥控映射
- IMU 索引
- 回环角设置

## 5. `Shoot_Task_Init()` 干了什么

位置：

- `TASK/Shoot_Task.c` 第 `596` 行

它会做这些事：

1. 清空整个发射任务状态
2. 默认摩擦轮模式设成 `STOP`
3. 默认拨盘模式设成 `STOP`
4. 注册左右摩擦轮 `M3508`
5. 注册拨盘 `LK`
6. 初始化摩擦轮速度环 PID
7. 初始化拨盘角度环和速度环 PID
8. 设置默认目标转速和默认步进角
9. 主动发一次 `LK` 状态查询，让在线判断尽快建立

当前默认硬件挂载是：

- 左摩擦轮：`CAN2 ID1 M3508`
- 右摩擦轮：`CAN2 ID2 M3508`
- 拨盘：`CAN2 ID5 LK`

## 6. 进入 `while(1)` 之后，主循环每圈做什么

主循环在 `Core/Src/main.c` 里当前就 3 件事：

1. `BSP_Poll()`
2. `Control_Task_Run()`
3. `Shoot_Task_Run()`

### 6.1 `BSP_Poll()`

位置：

- `BSP/BSP.c` 第 `262` 行

它会：

- 从 IMU 软件环形缓冲里取数据
- 调 `IMU_Process()`
- 从图传遥控软件环形缓冲里取数据
- 调 `Remote_Process()`

也就是说：

- 中断只负责搬运数据
- 真正解析协议，放在主循环里做

这样做的好处是：

- 中断短
- 不容易堵
- 协议解析不会占用中断时间

### 6.2 `Control_Task_Run()`

位置：

- `TASK/Control_Task.c` 第 `572` 行

它不是每次进主循环都完整跑一遍，而是先看：

- `gimbal_control.tick_pending`

只有 `TIM6` 中断给它加过节拍，它才真正执行一拍控制。

当前一拍的顺序是：

1. 取 `IMU_GetState()`
2. 取 `Remote_GetState()`
3. 刷新在线状态
4. 根据在线状态和遥控挡位决定当前模式
5. 如果 IMU 在线，就更新两轴反馈
6. 如果模式刚切换，做模式切换初始化
7. 按模式更新目标角
8. 做机械限位保护
9. 跑双环 PID
10. 发送两轴电流指令

### 6.3 `Shoot_Task_Run()`

位置：

- `TASK/Shoot_Task.c` 第 `672` 行

它也是一样，先看：

- `shoot_task_ctx.tick_pending`

只有有节拍时才真正执行。

当前一拍的顺序是：

1. 读 `Remote_GetState()`
2. 刷新在线状态
3. 决定摩擦轮模式
4. 处理摩擦轮模式切换
5. 更新左右摩擦轮反馈
6. 如果摩擦轮在运行，就跑速度环
7. 刷新 `friction_ready`
8. 决定拨盘模式
9. 处理拨盘模式切换
10. 处理遥控触发的拨盘步进请求
11. 如果允许闭环，就跑拨盘双环
12. 否则只做停机状态同步和在线轮询
13. 发送摩擦轮控制帧
14. 更新公开状态结构体

## 7. 中断链路是怎么和主循环配合的

## 7.1 `USART3_IRQHandler()`

位置：

- `Core/Src/stm32f4xx_it.c` 第 `301` 行

当前流程：

1. 串口 3 进中断
2. 先调用 `BSP_HandleUartIdle(&huart3)`
3. 如果是空闲中断，就把 DMA 新收到的数据搬到 IMU 软件环形缓冲
4. 再走 `HAL_UART_IRQHandler()`
5. 主循环里 `BSP_Poll()` 再把这段数据交给 `IMU_Process()`

## 7.2 `USART6_IRQHandler()`

位置：

- `Core/Src/stm32f4xx_it.c` 第 `387` 行

流程和 `USART3` 一样，只不过这里的数据最后会送去：

- `Remote_Process()`

## 7.3 `CAN1_RX0_IRQHandler()` / `CAN2_RX0_IRQHandler()`

位置：

- `CAN1_RX0_IRQHandler()`：`Core/Src/stm32f4xx_it.c` 第 `259` 行
- `CAN2_RX0_IRQHandler()`：`Core/Src/stm32f4xx_it.c` 第 `373` 行

流程是：

1. 中断里先进 `HAL_CAN_IRQHandler()`
2. HAL 最终回调到 `HAL_CAN_RxFifo0MsgPendingCallback()`
3. `BSP/BSP.c` 里把 FIFO0 里的消息一条条读出来
4. 交给 `Motor_ProcessCanMessage()`
5. `motor` 层按注册表找到对应电机对象并刷新反馈

所以现在电机反馈更新链路是：

`CAN 中断 -> HAL -> BSP 回调 -> motor 分发 -> 电机对象状态更新`

## 7.4 `TIM6_DAC_IRQHandler()`

位置：

- `Core/Src/stm32f4xx_it.c` 第 `345` 行

流程是：

1. `TIM6` 每 `1 ms` 进一次中断
2. 先进 `HAL_TIM_IRQHandler()`
3. HAL 最终回调到 `HAL_TIM_PeriodElapsedCallback()`
4. `BSP/BSP.c` 里调用：
   - `Control_Task_Timer1kHzCallback()`
   - `Shoot_Task_Timer1kHzCallback()`
5. 这两个函数只做一件事：
   - `tick_pending++`

注意：

- 它们不在中断里跑 PID
- 不在中断里发 CAN
- 不在中断里解析协议

真正的控制运算，还是放在主循环里。

## 8. 用一句话概括现在这套结构

当前程序不是“中断里直接跑全部控制”，而是：

- 中断负责收数据、收反馈、发节拍
- 主循环负责解析数据、跑控制、发输出

所以你可以把它理解成：

`中断是投递员，主循环是干活的人`

## 9. 你截图里的报错到底是什么

报错内容是：

`L6002U: Could not open file head_limit\startup_stm32f405xx.o: No such file or directory`

这条错误出现在链接阶段。

它的意思不是“找不到 `.s` 启动文件源文件”，而是：

- 链接器要去拿 `startup_stm32f405xx.o`
- 但是输出目录里没有这个目标文件

## 10. 我这边查到的实际情况

### 10.1 启动文件源文件是存在的

文件在：

- `MDK-ARM/startup_stm32f405xx.s`

### 10.2 Keil 工程里也确实把它加进去了

`MDK-ARM/Head_limit.uvprojx` 里能看到：

- `startup_stm32f405xx.s`

而且它在 `Application/MDK-ARM` 这个组里。

### 10.3 构建日志里也显示它被拿去编译了

`MDK-ARM/Head_limit/Head_limit.build_log.htm` 里能看到：

- `assembling startup_stm32f405xx.s...`

然后才进入：

- `linking...`

### 10.4 真正异常的是：输出目录里没有 `.o`

我查了 `MDK-ARM/Head_limit` 目录，当前只有：

- `.d`
- `.crf`
- `.lnp`
- `.sct`
- `.htm`

但没有看到：

- `startup_stm32f405xx.o`
- `main.o`
- 其他正常应该存在的 `.o`

所以这次不是“只有 startup 丢了”，而是：

- 这次构建没有把目标文件正常落盘
- 链接器只是先报了列表里的第一个缺失对象
- 第一个刚好是 `startup_stm32f405xx.o`

## 11. 这条错误最常见的原因

按你现在这个现象，最常见的是下面几种。

### 11.1 增量编译状态坏了

最像你现在这个。

表现是：

- Keil 觉得有些文件“已经编过”
- 但输出目录里的 `.o` 实际已经没了
- 最后链接时才报找不到对象文件

### 11.2 中间文件目录被清了，但没有彻底 Rebuild

比如：

- 你删过 `MDK-ARM/Head_limit` 里的部分文件
- 或者它被别的工具清理过
- 但 Keil 还是按增量构建流程在走

### 11.3 Keil 对目标文件输出有异常

比如：

- 某个工程配置状态坏了
- 文件选项异常
- 输出目录状态异常

这个概率比前两个小一些。

### 11.4 杀毒软件或权限问题拦了中间产物

这个概率更小，但也不是没有。

因为你这里 `.d` 和 `.crf` 能写进去，说明不是完全不能写。

## 12. 这条错误该怎么处理

建议按下面顺序做，不要一上来乱改代码。

### 第一步：直接 `Rebuild all target files`

不要点普通的 `Build`，要点：

- `Project -> Rebuild all target files`

因为你现在最像是增量编译缓存坏了。

### 第二步：如果还不行，关掉 Keil，清中间文件

删除目录：

- `MDK-ARM/Head_limit/`

里面的中间产物后，再重新打开工程做 `Rebuild all`。

建议删的主要是这些：

- `*.d`
- `*.crf`
- `*.dep`
- `*.lnp`
- `*.o`
- `*.axf`
- `*.map`
- `*.htm`

注意：

- 不要删 `MDK-ARM/Head_limit.uvprojx`
- 不要删 `MDK-ARM/startup_stm32f405xx.s`

### 第三步：确认启动文件没有被排除出构建

在 Keil 里看：

- `Application/MDK-ARM/startup_stm32f405xx.s`

右键：

- `Options for File...`

确认它没有被：

- `Exclude from Build`

虽然我这里看工程文件不像是这个问题，但还是值得顺手确认。

### 第四步：如果还不行，就删掉再重新把启动文件加回工程

做法是：

1. 从工程组里把 `startup_stm32f405xx.s` 移除
2. 再重新添加 `MDK-ARM/startup_stm32f405xx.s`
3. 重新 `Rebuild all`

### 第五步：最后再怀疑环境问题

如果前面都不行，再检查：

- Keil 是否有目录写权限
- 杀毒软件是否拦截 `.o`
- 工程路径是否被同步盘或别的工具占用

## 13. 你这次截图里的两个 warning 是什么

你现在还有两个 warning：

- `Motor_LimitFloat` 未被引用
- `Motor_UpdateTotalEcd` 未被引用

这两个只是：

- 有静态函数现在没被调用

它们不会导致这次链接失败。

这次真正让工程停下来的，只有：

- `L6002U`

## 14. 现在最推荐你先做什么

如果按优先级排，我建议你现在就做这两步：

1. 在 Keil 里执行一次 `Rebuild all target files`
2. 如果还报一样的错，就把 `MDK-ARM/Head_limit/` 里的中间产物清掉，再 `Rebuild all`

按你现在这个现象，这两步大概率就能解决。
