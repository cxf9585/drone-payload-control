# 无人机载荷控制模块

本模块用于 STM32 侧载荷控制，核心目标是把飞控命令、任务逻辑和具体硬件驱动分开。控制层不直接操作 GPIO、PWM、UART、CAN 等外设，所有硬件动作都通过 `payload_driver_t` 注入。

## 功能范围

- 载荷解锁、上锁和急停处理。
- 统一控制云台、相机、夹爪和喷洒类载荷。
- 通过 `payload_driver_t` 抽象硬件驱动。
- 提供常用 MAVLink `COMMAND_LONG` 命令映射。
- 提供主机侧单元测试，验证核心状态机行为。

## 目录结构

- `include/payload_control.h`：核心控制接口。
- `src/payload_control.c`：状态机、安全检查和命令调度。
- `include/payload_mavlink_bridge.h`：MAVLink 桥接接口。
- `src/payload_mavlink_bridge.c`：常用 MAVLink 命令映射。
- `examples/stm32_payload_app.c`：STM32 应用层接入示例。
- `tests/test_payload_control.c`：单元测试。
- `CMakeLists.txt`：主机侧构建配置。

## 接入方式

1. 引入 `include/payload_control.h`，并编译 `src/payload_control.c`。
2. 如果需要 MAVLink 命令转换，同时编译 `src/payload_mavlink_bridge.c`。
3. 按实际板级外设实现 `payload_driver_t` 中的函数，例如 GPIO、PWM、UART、CAN 或相机 SDK 调用。
4. 使用 `payload_init` 初始化控制器。
5. 使用 `payload_handle_command` 输入载荷命令。
6. 使用 MAVLink 时，先通过 `payload_command_from_mavlink_long` 把 `COMMAND_LONG` 转成内部命令。

`examples/stm32_payload_app.c` 展示了推荐的 STM32 接入方式。其中 TODO 函数是唯一应该直接访问硬件外设的位置。

## 安全默认行为

- 载荷动作在解锁前会被拒绝。
- `PAYLOAD_CMD_ESTOP` 会调用 `all_outputs_safe`，然后强制进入未解锁状态。
- 云台俯仰角、航向角和喷洒速率会先做范围检查，再调用硬件驱动。
- 上锁时如果提供了 `all_outputs_safe`，也会先执行安全输出。

## 需要补充的硬件信息

要把示例驱动改成真实驱动，需要明确以下信息：

- 载荷类型：云台、相机、夹爪、喷洒、绞盘、抛投或自定义载荷。
- 电气接口：GPIO、PWM、UART、CAN、RS485、I2C、SPI 或以太网。
- STM32 型号和板级引脚定义。
- 执行机构限制：舵机 PWM 范围、云台角度范围、泵最大电流或流量限制。
- 飞控链路：UART MAVLink、PX4 内部桥接、CAN 节点或其他协议。
- 失效保护策略：保持、停止、释放、闭合、断电或自定义安全动作。

## 构建和测试

当前目录提供了两种主机侧测试方式。

直接使用 C 编译器：

```sh
cc -Iinclude src/payload_control.c src/payload_mavlink_bridge.c tests/test_payload_control.c -o payload_control_test
./payload_control_test
```

使用 CMake：

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build
```

当前机器未安装 `gcc`、`clang`、MSVC `cl` 或 `cmake`，所以尚未在本机完成编译运行。
