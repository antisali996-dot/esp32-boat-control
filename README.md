# 小船及遥控器：2026-11-03 审查修订版

## 状态

源码已修改、复审，两端均完成 ESP32 Dev Module / Arduino-ESP32 3.0.2 编译。小船 Flash 936321 字节（71%）、RAM 46288 字节（14%）；遥控 Flash 904197 字节（68%）、RAM 43748 字节（13%）。小船只剩 ESP32Servo 库内未使用变量警告，遥控无警告。

本轮未烧录。板上仍运行之前的版本。新固件和源码快照分别在 firmware/boat-reviewed-20261103 与 firmware/remote-reviewed-20261103，清单 uploaded=false。原已烧录快照、原板 8MB 备份均保留。

修改前源码、说明及 SHA256 在 backup/before-audit-fixes-20261103-003242。

## 本轮修复

1. 电机无脉冲保护保留：输出达到 80 后持续 1.5 秒无脉冲锁定停机。新增高输出（PWM≥220）且转速低于目标 65% 持续 2 秒锁定停机，防止仍有脉冲的严重低速无限加油。锁定后需排查并重启。阈值未经实机整定，不等价于过流保护。
2. PI 积分可到达饱和边界，不再整步丢弃超限积分。启用时每控制周期最多增加 10，减速可直接降低。目标仍为 580 RPM；开机自动启动，遥控断开仍保持目标，按当前用户要求执行。
3. OLED 改用硬件 I2C，仍 SDA=26、SCL=27，100kHz，每笔事务超时 20ms。每秒检查 0x3C/0x3D 应答，重新出现时初始化；每 10 秒重新发送初始化/唤醒，覆盖短暂掉电发生在两次探测之间的情况。周期恢复可能带来短暂刷新闪动；ACK 不证明像素正常，无法修复屏幕损坏或电源故障。整帧通信仍在主循环，不能保证严格 50ms 控制周期；调速使用实际 elapsed 换算。
4. 接收端仅接受遥控器 MAC 24:62:AB:EF:CF:84。遥控目标为小船 24:62:AB:F0:72:94。双方信道1、八字节数据兼容。MAC 过滤不等价于加密认证。
5. 舵机改用明确脉宽：中心1500us，左右偏移333us，对应逻辑60～120。默认依据标称500～2500us/180°换算，工作脉宽限制1000～2000us。实际30°需校准 SERVO_SWING_US；不再将旧角度映射直接当作物理角度。
6. 遥控器在临界区内重新检查当前发送状态，消除旧 pending 快照引发的误停；回调超时仍禁发并要求重启。GPIO32 读摇杆，两个油门按键不参与当前自动定速功能。

## 保留行为及校准项

屏幕每秒显示模拟随机540～600，标记 SIM RPM，不用于控制。真实RPM、目标、PWM、收包及故障原因在115200串口。程序无主动熄屏逻辑，但不能保证硬件不黑屏。

PULSES_PER_REV=144 仍未标定：必须确认输出轴每转、单A相下降沿计数；未校准就不能确认物理580RPM。GPIO34 输入无内部上拉，接线、电平转换及抗干扰须按实际编码器验证。TS90A 电源应实测在允许范围；照片UBEC标称5V 3A，未认定原舵机实测电压为7.4V。

## 验证

持续维护的 tests/regression.cpp 直接包含实际 motor_control.h、servo_pulse.h、steering.h、tx_state.h，以g++编译运行通过：PI饱和可达255、超速降输出、持续低速停机、无脉冲停机、故障锁定、启动斜率、禁用输出、超时状态/时钟回绕、舵机脉宽、全4096个ADC值范围/单调性/死区连续性。

复审检查来源过滤在复制包之前执行、屏幕离线不阻止调速循环、超时判断与回调互斥、收发字段与信道一致。没有新增已确认的编译阻塞问题。

未进行带负载调速、无线联调、OLED断电恢复或舵机实际角度/温升验证。工作目录内保留编译缓存和日志用于复现，临时检查可执行文件已移入回收站。

## 仓库范围

Git 仓库维护小船及遥控器源码、回归测试和说明。原板 Flash、原版备份、已编译固件、工具链、依赖源码和运行日志仅保存在本地，不上传。README 中 backup、firmware、tools、work 路径均指本地工作目录。

## 在其他电脑编译

安装 Arduino CLI，并配置 Espressif 官方开发板索引：
https://espressif.github.io/arduino-esp32/package_esp32_index.json

依赖版本：Arduino-ESP32 3.0.2、ESP32Servo 3.2.1、U8g2 2.36.19。

```powershell
arduino-cli core update-index --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core install esp32:esp32@3.0.2 --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli lib install "ESP32Servo@3.2.1" "U8g2@2.36.19"
arduino-cli compile --fqbn esp32:esp32:esp32 boat
arduino-cli compile --fqbn esp32:esp32:esp32 remote
```

本机逻辑回归：使用 C++17 编译 tests/regression.cpp 后运行。修改代码后应重新测试、编译；旧编译记录不能代表新代码已通过验证。
