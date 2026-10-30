# 两轮自平衡小车

基于 **Quectel Pi M2 加 STM32** 实现的两轮自平衡小车 Demo。

STM32 负责姿态检测、电机驱动及平衡控制，Quectel Pi M2 负责 Web 控制和 UART 通信，手机与 M2 连接至同一 Wi-Fi 网络后，可通过浏览器访问控制页面，实现小车前进、后退及转向控制。


## 功能特性

- MPU6050 姿态检测与角度融合
- 两轮自平衡控制
- 编码器电机速度检测
- PID直立环、速度环和转向环控制
- M2 与 STM32 UART 通信
- Web 页面远程控制
- 支持前进、后退、停止及左右转向
- 支持目标速度和转向角速度控制

## 硬件组成

| 硬件 | 说明 |
| --- | --- |
| Quectel Pi M2 | Web 控制及 UART 通信 |
| STM32F103C8T6 | 小车实时控制 |
| MPU6050 | 六轴 IMU，获取加速度及角速度 |
| 编码器减速电机 | 小车驱动及速度反馈 |
| TB6612 | 双路直流电机驱动 |
| OLED | 状态信息显示 |

## 项目结构

```text
demo_balance_car/
├── balance_car_m2/                    # Quectel Pi M2 端程序
│   ├── include/
│   │   ├── car_control.h              # 运动目标与控制接口
│   │   └── uart.h                     # 串口通信与运动指令接口
│   ├── src/
│   │   ├── car_web.c                  # 内嵌网页、HTTP 接口与服务入口
│   │   ├── car_control.c              # 速度、转向和停车指令封装
│   │   └── uart.c                     # Linux 串口配置与控制帧发送
│   ├── Makefile                       # M2 端构建规则
│   └── PROJECT.md                     # M2 端开发阶段记录
│
├── balance_car_stm32/                 # STM32 实时控制程序
│   ├── Core/
│   │   ├── Inc/                      # 控制模块及外设接口头文件
│   │   └── Src/
│   │       ├── main.c                # 外设初始化与主循环入口
│   │       ├── balance_car.c         # 控制任务、定时中断与安全停机
│   │       ├── balance_controller.c  # 姿态直立控制
│   │       ├── speed_controller.c    # 编码器速度 PI 控制
│   │       ├── turn_controller.c     # 偏航角速度转向控制
│   │       ├── attitude.c            # 陀螺仪校准与姿态角融合
│   │       ├── mpu6050.c             # MPU6050 初始化与数据读取
│   │       ├── motor.c               # 双电机方向、PWM 与停机控制
│   │       ├── encoder.c             # 双轮编码器计数与转速计算
│   │       ├── pid.c                 # 通用 PID 计算与限幅
│   │       ├── key.c                 # 平衡开关状态读取
│   │       ├── oled.c                # OLED 状态显示
│   │       ├── system_init.c         # 系统时钟及错误处理
│   │       └── usart.c               # USART1 初始化与控制帧解析
│   ├── Drivers/                       # STM32 HAL 与 CMSIS
│   ├── MDK-ARM/                       # Keil 工程及启动汇编源码
│   ├── balance_car_stm32.ioc          # STM32CubeMX 配置
│   └── .mxproject                     # STM32CubeMX 生成记录
│
├── .gitignore
└── README.md
```

## 快速运行

### STM32

使用 Keil MDK 打开：

```text
balance_car_stm32/MDK-ARM/balance_car_stm32.uvprojx
```
编译后通过 ST-Link 将程序烧录至 STM32。
### Quectel Pi M2
在 M2 环境中进入 balance_car_m2 目录并编译运行：
```
make car_web
./car_web
```
程序默认监听 0.0.0.0:8080。手机与 M2 连接至同一 Wi-Fi 网络后，在浏览器访问：
http://<M2_IP>:8080

即可进入小车 Web 控制页面。

## Web 控制

Web 页面通过 HTTP 将控制指令发送至 M2，M2 再通过 UART 将目标速度和目标转向角速度发送给 STM32。
速度采用点按控制，转向采用按住控制，松开转向按钮后，目标转向角速度恢复为 `0 °/s`。

## 控制参数

主要控制参数位于 STM32 工程中：

| 参数 | 作用 |
| --- | --- |
| `BALANCE_CAR_BALANCE_ANGLE_DEG` | 机械平衡角 |
| `BalancePID.config.kp/kd` | 直立环参数 |
| `SpeedPID.config.kp/ki` | 速度环参数 |
| `TurnKp` | 转向环参数 |
| `target_speed_rpm` | 目标速度 |
| `target_yaw_rate_dps` | 目标转向角速度 |

建议按照 **直立环 → 速度环 → 转向环** 的顺序进行参数调试。

## 注意事项

- 调试前确认 MPU6050 安装方向正确。
- 首次测试建议将车轮悬空，确认电机方向正确。
- PID 参数应逐步调整，避免一次修改幅度过大。
- 手机与 M2 应保持稳定的 Wi-Fi 连接，网络质量较差时 Web 控制可能出现延迟。
- 修改 STM32 程序后，需要重新编译并通过 ST-Link 烧录。

## 报告问题
如在使用过程中遇到问题，欢迎在移远官方论坛提交技术咨询，我们的技术支持团队将及时为您解答。 欢迎提交Issue和Pull Request来改进此项目。