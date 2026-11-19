飞镖单发考核代码交付说明（2026-11-16）
适用目录：E:\RM\dart_skeleton。仅使用本目录考核说明、组件和手册。

一、实现范围
1. M3508/M2006 设备类：Dart-Components/control/motor.hpp。
   大端反馈解析、有符号转速/电流、多圈累计、方向、减速比、输出角速度、电流单位换算、输入限幅及0x200分段打包。
   ID1右带正向，ID2左带反向，ID3丝杆反向。角度转换按考核2π/8191；跨零按8192编码范围判断。
   反馈50ms失联；反馈间隔>=10ms重新建立多圈段并增加epoch，正在运动的状态机检测到epoch变化即停止并要求重新回零。
   编码器为相对位置源，断电后无法恢复机械绝对零点；第一次触发及中止后都重新回零。
2. CAN1：Task/can_feedback.cpp。1Mbps、FIFO0全接收过滤、标准数据帧软件分发、统一0x200发送、第四段清零。
   最终发送前再次检查有效遥控、双下、持续内八、三台在线、输出许可及CAN状态。
   记录发送成功/错误/等待，待发超过10ms置故障；退出动作时中止旧待发帧并发送零帧。
   CAN物理断线时无法保证零帧送达，需依赖电调通信超时及实机停止能力验收。
3. DBUS：既有DT7保持不改。Task/comm_task.cpp采用18字节普通DMA，TC满帧解码，IDLE短帧拒绝后重启，HT禁用。
   在通讯初始化中将当前CubeMX生成的循环DMA切换为普通DMA；不需重新Generate Code。
   TC满帧可正常处理，不再沿用旧36字节版本“丢弃TC”的策略。半包/错误后等待下一帧边界恢复；DBUS无CRC，校验不能发现所有误码。
4. PID类：Dart-Components/control/pid.hpp。位置式PID，积分与输出限幅、条件积分抗饱和、复位、异常输入处理。
5. 同步带：Dart-Components/modules/belt。用户说明无已知限位引脚，采用低速限流堵转回零。
   回零必须由Working态有效遥控双下+内八触发，不能上电自动转动。回零速度、最大位移、时间、反馈电流与持续低速均有约束。
   两边分别确认到端点后停止该侧、设相对零；随后双位置/速度环。带到位稳定时间、左右偏差、软行程、运动超时和堵转保护。
6. 丝杆：Dart-Components/modules/screw。触发时捕获当前位置作为预紧位，在本次动作中保持，禁止自动寻找无硬件限位的丝杆端点。
   限速/限流/最大位置误差、离线与堵转停止。未实现非必需的遥控微调加分项。
7. 舵机：Dart-Components/modules/shoot 与 Instance/motor_ins.cpp。
   复用提供的Servo，95°安全/180°击发；击发保持1000ms，复位等待300ms。
   已提供参数50..250在现有100kHz定时器实际对应500..2500us。驱动内部ARR计算却假定1MHz。
   实例适配层保持现有PSC与脉宽计数，将传给驱动的频率乘10，使实际PWM为配置要求的200Hz；不修改提供的Servo组件。
   若实际定时器不是100kHz则拒绝使能。舵机型号是否支持该脉宽与200Hz仍需实测确认。
8. 顶层：MainFsm继承提供的Fsm，生命状态基类未改。纯逻辑Sequence负责动作，Task进行硬件读写。
9. 调度：TIM3每2ms执行MainTask和CommTask。正式模式只在有效遥控帧且调度推进时喂IWDG；诊断模式则由完整控制周期喂狗。

二、动作顺序
进入Working -> 双下档先松开摇杆 -> 保持内八 -> 堵转回零 -> 移到-5mm准备位 -> 拉下-673mm -> 拉起-5mm -> 舵机180°保持1s -> 回95° -> 等待松开。
必须全过程保持内八（任务书第8页）。任何阶段松开/切安全档/遥控离线/未Working/输出未许可均停止输出、舵机回安全角。
保持内八完成一发后不连发，松开再内八才开始下一次。每次重新触发重新回零，避免在相对编码器失去可信零点后继续使用旧位置。
电机离线、编码器多圈段变化、堵转、超时、超行程、CAN故障锁存。必须退出操作档位清除，恢复反馈不能自动继续动作。

三、构建模式与标定入口
默认命令：
 cmake --preset Debug
 cmake --build --preset Debug
 离线测试目录已按用户要求移除；此命令不再适用。
默认产物 build/Dart2025.elf：DART_DIAGNOSTIC_MODE=ON，DART_HARDWARE_CALIBRATED=OFF。
诊断模式不发送电机控制帧、不启动舵机PWM，可观察通信；不满足正式“关遥控器约1秒复位”验收。

控制参数集中在 Dart-Configs/control_config.hpp。PID、堵转电流/速度/时间阈值、运动速度、行程与超时均为待标定初值。
没有从未知的硬件中推断限位引脚，也没有假称这些PID经过实机调试。
堵转只能判断“停止且有负载”，不能区分真正机械端点与途中障碍；回零路径必须确认无障碍，机械端点须能承受受限回零力。
当前回零电流上限原始1200约1.46A，判据1A且速度<1mm/s持续400ms，初始等待500ms；时间80s、最大相对行程750mm。实际结构必须重新确认这些值。
实机参数及方向验证完成后，正式模式的构建开关为：
 DART_DIAGNOSTIC_MODE=OFF
 DART_HARDWARE_CALIBRATED=ON
开关属于CMake缓存选项；只改头文件默认值不会覆盖CMake传入的宏。必须重新配置并构建。
本次仅编译了 build/VerifyOperational 中的上述正式分支作为构建验证，未下载，不能将“CALIBRATED=ON用于编译检查”当作真实硬件已标定。
默认build/Dart2025.elf仍为输出禁用诊断版。

四、调试变量
g_rc_debug：原遥控字段。
g_rc_diag/g_rc_previous：当前/前一轮暖复位诊断，DMA初始长度现在是18；previous记录有版本校验。
g_can_debug：接收、电机在线掩码、发送计数/失败/命令。
g_safety_debug：模式、生命状态、动作输出许可。
g_control_debug：step、fault、hardware_ready、左右带位置、舵机目标。
step：0 Standby，1 Ready，2 Homing，3 Positioning，4 Pulldown，5 Pullup，6 Shoot，7 Finish，8 WaitRelease，9 Fault。
fault：0 None，1 Offline，2 FeedbackReset，3 HomeTimeout，4 HomeTravel，5 Stall，6 Skew，7 Travel，8 MoveTimeout，9 ScrewTravel，10 SequenceTimeout，11 Can，12 Servo（预留）。
Servo故障在硬件许可门控中阻止启动，观察hardware_ready及PWM配置。

五、验证结果与未完成的实机验收
默认诊断分支和正式输出分支均编译链接成功；仍有既有RWX LOAD段链接警告。
三个编译期测试组通过：遥控帧/离线/边沿；电机反馈协议；PID/多圈过零/方向/控制编码/完整单发时序/每个阶段的松手、模式退出、未Working、硬件禁用、电机离线、CAN故障/故障锁存/回零超时和异常行程。
测试执行的是C++ constexpr逻辑断言，不是机械动力学仿真或实机测试；理想反馈只验证流程，不证明PID稳定性。
尚需实机验收：遥控接收链路恢复、三台电机CAN反馈、方向和零点、堵转回零阈值、PID稳定性、舵机脉宽、急停/离线/看门狗复位。
未烧录、未运行机械动作、未提交或推送Git。目录内未发现Git仓库，GitHub交付需后续按用户指定仓库安排。
