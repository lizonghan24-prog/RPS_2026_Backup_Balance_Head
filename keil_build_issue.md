# Keil 当前编译问题排查记录

## 1. 当前报错

当前链接报错是：

`L6002U: Could not open file head_limit\startup_stm32f405xx.o: No such file or directory`

表面上看像是：

- 启动文件 `startup_stm32f405xx.s` 没找到

但实际上不是。

## 2. 实际查到的情况

### 2.1 启动文件源码是存在的

当前文件在：

- `MDK-ARM/startup_stm32f405xx.s`

### 2.2 Keil 工程里也已经把它加进去了

在 `MDK-ARM/Head_limit.uvprojx` 里可以看到：

- `startup_stm32f405xx.s`

### 2.3 Build Log 里也显示它参与了构建

日志里明确有：

- `assembling startup_stm32f405xx.s...`

### 2.4 真正异常的是：没有任何 `.o` 产物

当前 `MDK-ARM/Head_limit/` 目录里有：

- `.d`
- `.crf`
- `.lnp`
- `.dep`
- `.build_log.htm`

但是没有：

- `startup_stm32f405xx.o`
- `main.o`
- `bsp.o`
- 其他正常应该出现的目标文件

所以这次不是“只有 startup 的 `.o` 丢了”，而是：

- 整个工具链都没有正常产出目标文件

## 3. 已经验证过的结论

### 3.1 不是代码路径问题

因为：

- 启动文件路径存在
- 工程文件路径存在
- 依赖文件 `.dep` 里也能看到正确的 `-o head_limit\\xxx.o`

### 3.2 不是目录写权限问题

因为手工在同一目录创建假的 `.o` 文件是成功的。

也就是说：

- `MDK-ARM/Head_limit/` 这个目录本身是能写的

### 3.3 不是单纯的 startup 文件问题

因为不只是 `startup_stm32f405xx.o` 没有生成，

连：

- `main.o`
- `control_task.o`
- `shoot_task.o`

这些 C 文件目标也没有生成。

### 3.4 更像是工具链在“产出目标文件”这一步失效了

目前看到的现象是：

- `Armcc.exe --vsn` 能正常返回版本
- `Armlink.exe --vsn` 能正常返回版本
- 但 `Armasm.exe --vsn` 直接失败
- 手工调用 `Armcc.exe` 做真正编译时，也不会产出 `.o`

这说明问题更像是：

- ARMCC / ARMASM 代码生成阶段异常

而不是：

- 你的工程代码有普通语法错误

## 4. 现在最值得怀疑的方向

按优先级排序，当前最值得怀疑的是这些。

### 4.1 ARM Compiler 5 安装或运行环境异常

这是当前最像的原因。

因为：

- 版本查询可以工作
- 但真正编译和汇编阶段不能正常产出目标文件

### 4.2 ARMASM 工具本身状态异常

当前 `startup` 是汇编启动文件，必须依赖 `Armasm.exe`。

而我这边直接调用：

- `Armasm.exe --vsn`

就已经失败了。

这很不正常。

### 4.3 安全软件拦截了编译器真正的目标文件输出

也有可能。

因为现在的现象是：

- `.d` 和 `.crf` 能写
- `.o` 却没有出来

这种情况有时是安全软件对编译器行为做了拦截。

### 4.4 Keil / ARMCC 许可状态异常

也不能完全排除。

虽然 IDE 日志里能看到许可信息，
但实际代码生成阶段如果授权链路不完整，也可能出现非常怪的现象。

## 5. 你现在应该怎么做

## 5.1 先不要继续怀疑业务代码

当前最重要的判断是：

- 这不是 `Control_Task.c` 或 `Shoot_Task.c` 里某个语法错误导致的普通编译失败
- 现在连 `.o` 都没生成出来，问题在更前面

## 5.2 在 Keil 里先检查 License Management

打开：

- `File -> License Management`

重点确认：

- `MDK Plus` 是否正常激活
- `ARM Compiler 5` 是否可用

## 5.3 修复或重装 ARM Compiler 5

如果你最近动过 Keil、Pack、编译器版本或者安装目录，
我建议直接做一次修复安装。

优先保证：

- `C:\\Keil_v5\\ARM\\ARMCC\\Bin\\armcc.exe`
- `C:\\Keil_v5\\ARM\\ARMCC\\Bin\\armasm.exe`
- `C:\\Keil_v5\\ARM\\ARMCC\\Bin\\armlink.exe`

这一套是完整可用的。

## 5.4 把 Keil 和 ARMCC 目录加入安全软件白名单

如果你开了安全软件、Windows Defender 的受控文件夹访问之类的东西，
建议把下面这些路径加入白名单：

- `C:\\Keil_v5\\UV4\\`
- `C:\\Keil_v5\\ARM\\ARMCC\\Bin\\`
- 当前工程目录 `C:\\Users\\makabaka\\Desktop\\Head_limit\\`

## 5.5 重新做一次完整 Rebuild

在处理完上面几项以后，再做一次：

- `Project -> Rebuild all target files`

重点看：

- `MDK-ARM/Head_limit/` 里有没有开始出现 `.o`

只要 `.o` 能正常出来，链接阶段这条 `L6002U` 基本就会消失。

## 6. 现在不建议做的事

当前不建议你先去做这些：

- 继续乱改业务代码
- 继续换启动文件名字
- 继续手工改链接脚本
- 继续怀疑 PID、保护逻辑、任务逻辑本身

因为根因现在更像：

- 编译器没有正常吐目标文件

## 7. 这次代码层面我顺手做的事

为了让后面继续排查更干净，我已经顺手做了这些整理：

- 给 `Control_Task.c` 里保护和模式切换部分补了更细的中文注释
- 给 `Shoot_Task.c` 里摩擦轮联锁、拨盘闭环和保护逻辑补了更细的中文注释
- 把 `motor.c` 里两个没用到的旧静态函数删掉了，减少无关 warning

所以你后面再次编译时，输出里会比之前更干净一点。
