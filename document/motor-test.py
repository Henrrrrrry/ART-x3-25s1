import time
from machine import I2C

# PCA9685寄存器地址
PCA9685_ADDR = 0x40
MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06

i2c = I2C(2)
print("I2C扫描结果:", i2c.scan())

def reset_pca9685():
    # 软复位PCA9685
    try:
        # 读取当前MODE1寄存器值
        mode1 = i2c.readfrom_mem(PCA9685_ADDR, MODE1, 1)[0]
        print(f"当前MODE1值: 0x{mode1:02x}")

        # 设置睡眠模式
        i2c.writeto_mem(PCA9685_ADDR, MODE1, bytes([mode1 | 0x10]))

        # 设置PWM频率 (50Hz)
        prescale = 25000000 // (4096 * 50) - 1
        i2c.writeto_mem(PCA9685_ADDR, PRESCALE, bytes([prescale]))

        # 恢复正常模式
        i2c.writeto_mem(PCA9685_ADDR, MODE1, bytes([mode1 & ~0x10]))
        time.sleep(0.005)  # 等待振荡器启动

        # 启用自动增量
        i2c.writeto_mem(PCA9685_ADDR, MODE1, bytes([mode1 & ~0x10 | 0xa0]))

        print("PCA9685重置完成")
        return True
    except Exception as e:
        print("PCA9685重置失败:", e)
        return False

def set_servo_pulse(channel, pulse_us):
    # 将脉冲宽度(微秒)转换为12位计数值
    # 50Hz = 20ms周期，4096个计数点
    pulse_count = pulse_us * 4096 // 20000

    # 计算寄存器值
    channel_base = LED0_ON_L + (channel * 4)

    # 设置PWM开始点和结束点
    data = [
        0, 0,                           # LED_ON值 (低字节和高字节)
        pulse_count & 0xFF,             # LED_OFF低字节
        (pulse_count >> 8) & 0x0F       # LED_OFF高字节
    ]

    # 写入寄存器
    i2c.writeto_mem(PCA9685_ADDR, channel_base, bytes(data))
    print(f"设置通道{channel}脉宽为{pulse_us}微秒 (值={pulse_count})")

# 测试FS90R连续旋转舵机
try:
    if reset_pca9685():
        channel = 0  # S0通道
        print("\n测试FS90R连续旋转舵机")

        # 停止状态
        print("停止电机")
        set_servo_pulse(channel, 1500)  # 中间位置 - 停止
        time.sleep(3)

        # 正向旋转 - 慢速
        print("正向旋转 - 慢速")
        set_servo_pulse(channel, 1600)  # 稍高于中间值
        time.sleep(3)

        # 正向旋转 - 中速
        print("正向旋转 - 中速")
        set_servo_pulse(channel, 1700)
        time.sleep(3)

        # 正向旋转 - 快速
        print("正向旋转 - 快速")
        set_servo_pulse(channel, 2000)  # 最高值
        time.sleep(3)

        # 停止
        print("停止电机")
        set_servo_pulse(channel, 1500)
        time.sleep(3)

        # 反向旋转 - 慢速
        print("反向旋转 - 慢速")
        set_servo_pulse(channel, 1400)  # 稍低于中间值
        time.sleep(3)

        # 反向旋转 - 中速
        print("反向旋转 - 中速")
        set_servo_pulse(channel, 1300)
        time.sleep(3)

        # 反向旋转 - 快速
        print("反向旋转 - 快速")
        set_servo_pulse(channel, 1000)  # 最低值
        time.sleep(3)

        # 停止
        print("停止电机")
        set_servo_pulse(channel, 1500)

        print("测试完成")

except Exception as e:
    import sys
    print("发生错误:", e)
    sys.print_exception(e)

