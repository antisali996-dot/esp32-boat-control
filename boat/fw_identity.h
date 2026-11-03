#pragma once

// 固件身份的唯一来源（小船与遥控器共用，本文件在两处必须完全一致）。
//
// 设计目的：
//   1. FWID 由实际参与控制的常量拼出，改角度、脉宽、死区或转速，FWID 必然变化；
//   2. 工具只认 FWID 与本包固件清单完全一致，才判定“最新”；
//   3. 配对协议版本（VERSION=）只描述串口命令兼容性，禁止用来判断固件新旧；
//   4. 旧 30° 程序没有 FWID，工具必须判定为“需要升级”，不允许跳过。
//
// 修改本文件后必须重新编译两端并重新生成固件清单，禁止只改说明。
// servo_pulse.h / steering.h 以 static_assert 把本文件的数值钉死，
// 任何一边改数值而另一边没改，编译阶段就会失败。

#define FW_SCHEMA 3
#define FW_VER "20261103-20deg-r3"

// 转向：中位 90°，逻辑范围 70～110°（左右各 20°）。
#define FW_STEER_DEG 20
#define FW_CENTER_ANGLE 90
#define FW_MIN_ANGLE 70
#define FW_MAX_ANGLE 110

// 舵机脉宽：按标称 500～2500us/180° 换算，20° 约 222us。
// 数值为实际下发脉宽，必须与 SERVO_CENTER_US / SERVO_SWING_US 保持一致。
#define FW_CENTER_US 1500
#define FW_SWING_US 222
#define FW_MIN_US 1278
#define FW_MAX_US 1722

// 调速策略保持既有 580 RPM 与全部保护逻辑，这里只登记身份。
#define FW_TARGET_RPM 580
#define FW_CONTROL_REV "580pi2"

// 遥控器摇杆死区（沿用原程序数值）。
#define FW_JOY_DEAD_LOW 1750
#define FW_JOY_DEAD_HIGH 2150

// 配对协议版本（JAY INFO / JAY BIND 命令格式）；与固件新旧无关。
#define FW_PAIR_PROTOCOL 2

#define FW_STR_(x) #x
#define FW_STR(x) FW_STR_(x)

#define FW_ID_STEER "S" FW_STR(FW_STEER_DEG) "-A" FW_STR(FW_MIN_ANGLE) "-" FW_STR(FW_MAX_ANGLE) \
                   "-U" FW_STR(FW_MIN_US) "-" FW_STR(FW_CENTER_US) "-" FW_STR(FW_MAX_US)

// 小船：转向契约 + 目标转速 + 控制修订号。
#define FW_ID_BOAT "JAYFWB" FW_STR(FW_SCHEMA) "-" FW_ID_STEER \
                   "-R" FW_STR(FW_TARGET_RPM) "-C" FW_CONTROL_REV

// 遥控器：转向契约 + 摇杆死区。
#define FW_ID_REMOTE "JAYFWR" FW_STR(FW_SCHEMA) "-" FW_ID_STEER \
                     "-D" FW_STR(FW_JOY_DEAD_LOW) "-" FW_STR(FW_JOY_DEAD_HIGH)
