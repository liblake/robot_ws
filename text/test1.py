#!/usr/bin/env python3
"""
手柄检测程序
使用 inputs 库读取 Linux 游戏手柄的实时状态。
按 Ctrl+C 退出。
"""

import sys

try:
    import inputs
except ImportError:
    print("错误：请先安装 inputs 库：")
    print("    pip install inputs")
    sys.exit(1)


def main():
    # 1. 检测已连接的手柄
    gamepads = inputs.devices.gamepads
    if not gamepads:
        print("❌ 未检测到手柄。")
        print("请检查：")
        print("  1. 手柄是否已通过 USB 或蓝牙连接？")
        print("  2. 是否有权限访问 /dev/input/js* ？")
        print("     可以尝试：sudo usermod -a -G input $USER  然后重新登录。")
        print("  3. 或者运行：sudo python3 test_gamepad.py （临时测试用）")
        return

    # 取第一个手柄
    pad = gamepads[0]
    # 修正：直接使用 pad.path 和 pad.name，没有 device 属性
    print(f"✅ 检测到手柄: {pad.path} ({pad.name})")
    print("开始读取事件（摇动摇杆或按下按键），按 Ctrl+C 退出。\n")

    try:
        while True:
            events = pad.read()
            for event in events:
                if event.ev_type == 'Absolute':  # 轴事件（摇杆、扳机）
                    print(f"轴 {event.code:>3} : {event.state:>6}")
                elif event.ev_type == 'Key':     # 按钮事件（按下/释放）
                    state_str = "按下" if event.state else "释放"
                    print(f"按钮 {event.code:>3} : {state_str}")

    except KeyboardInterrupt:
        print("\n退出程序。")
    except Exception as e:
        print(f"读取手柄时出错: {e}")


if __name__ == "__main__":
    main()