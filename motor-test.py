"""
SG90R Continuous Rotation Servo Motor Test System

Authors: 
    Author 1: [Qianwen Shen]: Implement basic testing system
    Author 2: [Leliang Wang]: Implement basic testing system
    Author 3: [Hongyu Li]: Change angle control into pulse control, add two more motors' test

Description:
This system tests four SG90R continuous rotation servo motors connected to PCA9685 PWM driver.
Motors are grouped for synchronized movement testing.

Hardware Requirements:
- OpenMV Camera (H7/M7)
- PCA9685 PWM Driver Board
- 4x SG90R Continuous Rotation Servo Motors
- I2C connections between OpenMV and PCA9685

Motor Configuration:
- Group 1: S0 and S2 motors (synchronized movement)
- Group 2: S1 and S3 motors (synchronized movement)

Input:
- I2C communication with PCA9685
- PWM control signals

Output:
- PWM signals to servo motors
- Serial console feedback for testing status

Functions:
- reset_pca9685(): Initialize PCA9685 PWM driver
- set_servo_pulse(): Send PWM signals to specific servo channel
- test_motor_group(): Test synchronized motor group movement
"""

import time
from machine import I2C

# ============================================================================
# PCA9685 PWM DRIVER CONFIGURATION
# ============================================================================
# PCA9685 register addresses for I2C communication
PCA9685_ADDR = 0x40    # I2C address of PCA9685
MODE1 = 0x00           # Mode register 1
PRESCALE = 0xFE        # Prescaler register for PWM frequency
LED0_ON_L = 0x06       # First PWM channel register

# Initialize I2C communication
i2c = I2C(2)
print("I2C scan result:", i2c.scan())

# ============================================================================
# PCA9685 INITIALIZATION FUNCTION
# ============================================================================
def reset_pca9685():
    """
    Initialize PCA9685 PWM driver for servo control
    Sets up 50Hz PWM frequency suitable for SG90R servos
    
    Input: None
    Output: Boolean - True if initialization successful, False if failed
    """
    try:
        # Read current MODE1 register value
        mode1 = i2c.readfrom_mem(PCA9685_ADDR, MODE1, 1)[0]
        print(f"Current MODE1 value: 0x{mode1:02x}")

        # Set sleep mode for configuration
        i2c.writeto_mem(PCA9685_ADDR, MODE1, bytes([mode1 | 0x10]))

        # Set PWM frequency to 50Hz (standard for servos)
        # Formula: prescale = 25MHz / (4096 * frequency) - 1
        prescale = 25000000 // (4096 * 50) - 1
        i2c.writeto_mem(PCA9685_ADDR, PRESCALE, bytes([prescale]))

        # Wake up from sleep mode
        i2c.writeto_mem(PCA9685_ADDR, MODE1, bytes([mode1 & ~0x10]))
        time.sleep(0.005)  # Wait for oscillator to start

        # Enable auto-increment mode
        i2c.writeto_mem(PCA9685_ADDR, MODE1, bytes([mode1 & ~0x10 | 0xa0]))

        print("PCA9685 reset completed successfully")
        return True
    except Exception as e:
        print("PCA9685 reset failed:", e)
        return False

# ============================================================================
# SERVO CONTROL FUNCTION
# ============================================================================
def set_servo_pulse(channel, pulse_us):
    """
    Send PWM pulse to specific servo channel
    Converts microsecond pulse width to 12-bit PWM value
    
    Input: 
        channel (int) - Servo channel number (0-15)
        pulse_us (int) - Pulse width in microseconds (1000-2000)
    Output: None (sends PWM signal to servo)
    """
    # Convert pulse width (microseconds) to 12-bit count value
    # 50Hz = 20ms period, 4096 count points
    pulse_count = pulse_us * 4096 // 20000

    # Calculate register base address for this channel
    channel_base = LED0_ON_L + (channel * 4)

    # Prepare PWM data (ON time = 0, OFF time = pulse_count)
    data = [
        0, 0,                           # LED_ON value (low and high bytes)
        pulse_count & 0xFF,             # LED_OFF low byte
        (pulse_count >> 8) & 0x0F       # LED_OFF high byte
    ]

    # Write to PCA9685 registers
    i2c.writeto_mem(PCA9685_ADDR, channel_base, bytes(data))
    print(f"Set channel {channel} pulse width to {pulse_us}us (value={pulse_count})")

# ============================================================================
# MOTOR GROUP CONTROL FUNCTION
# ============================================================================
def test_motor_group(group_name, channel1, channel2, stop_pulse):
    """
    Test synchronized movement of two motors in a group
    
    Input:
        group_name (str) - Name of the motor group for logging
        channel1 (int) - First motor channel
        channel2 (int) - Second motor channel
        stop_pulse (int) - Pulse width for motor stop position
    Output: None (controls motor movement and prints status)
    """
    print(f"\nTesting {group_name}")

    # Ensure both motors are stopped initially
    print(f"{group_name}: Both motors stop")
    set_servo_pulse(channel1, stop_pulse)
    set_servo_pulse(channel2, stop_pulse)
    time.sleep(2)

    # Forward rotation test
    print(f"{group_name}: Both motors forward rotation")
    set_servo_pulse(channel1, 1700)  # Forward direction
    set_servo_pulse(channel2, 1700)  # Forward direction
    time.sleep(3)

    # Stop motors
    print(f"{group_name}: Both motors stop")
    set_servo_pulse(channel1, stop_pulse)
    set_servo_pulse(channel2, stop_pulse)
    time.sleep(2)

    # Reverse rotation test
    print(f"{group_name}: Both motors reverse rotation")
    set_servo_pulse(channel1, 1300)  # Reverse direction
    set_servo_pulse(channel2, 1300)  # Reverse direction
    time.sleep(3)

    # Final stop
    print(f"{group_name}: Both motors stop")
    set_servo_pulse(channel1, stop_pulse)
    set_servo_pulse(channel2, stop_pulse)
    time.sleep(2)

# ============================================================================
# MAIN TESTING SEQUENCE
# ============================================================================
# SG90R servo motor parameters
STOP_PULSE = 1520      # Precise stop point for SG90R motors
FORWARD_PULSE = 1700   # Forward rotation pulse width
REVERSE_PULSE = 1300   # Reverse rotation pulse width

# Motor channel assignments
GROUP1_MOTOR1 = 0      # S0 channel
GROUP1_MOTOR2 = 2      # S2 channel
GROUP2_MOTOR1 = 1      # S1 channel
GROUP2_MOTOR2 = 3      # S3 channel

# Execute motor testing sequence
try:
    if reset_pca9685():
        print("\nTesting four SG90R continuous rotation servo motors")
        print("Motor grouping:")
        print("  Group 1: S0 and S2 motors (synchronized)")
        print("  Group 2: S1 and S3 motors (synchronized)")

        # Test Group 1 (S0 and S2)
        test_motor_group("Group 1 (S0 & S2)", GROUP1_MOTOR1, GROUP1_MOTOR2, STOP_PULSE)

        # Test Group 2 (S1 and S3)
        test_motor_group("Group 2 (S1 & S3)", GROUP2_MOTOR1, GROUP2_MOTOR2, STOP_PULSE)

        # Test both groups simultaneously
        print("\nTesting both groups simultaneously")

        # All motors forward
        print("All motors: Forward rotation")
        set_servo_pulse(GROUP1_MOTOR1, FORWARD_PULSE)  # S0
        set_servo_pulse(GROUP1_MOTOR2, FORWARD_PULSE)  # S2
        set_servo_pulse(GROUP2_MOTOR1, FORWARD_PULSE)  # S1
        set_servo_pulse(GROUP2_MOTOR2, FORWARD_PULSE)  # S3
        time.sleep(3)

        # All motors stop
        print("All motors: Stop")
        set_servo_pulse(GROUP1_MOTOR1, STOP_PULSE)
        set_servo_pulse(GROUP1_MOTOR2, STOP_PULSE)
        set_servo_pulse(GROUP2_MOTOR1, STOP_PULSE)
        set_servo_pulse(GROUP2_MOTOR2, STOP_PULSE)
        time.sleep(2)

        # All motors reverse
        print("All motors: Reverse rotation")
        set_servo_pulse(GROUP1_MOTOR1, REVERSE_PULSE)  # S0
        set_servo_pulse(GROUP1_MOTOR2, REVERSE_PULSE)  # S2
        set_servo_pulse(GROUP2_MOTOR1, REVERSE_PULSE)  # S1
        set_servo_pulse(GROUP2_MOTOR2, REVERSE_PULSE)  # S3
        time.sleep(3)

        # Final stop for all motors
        print("All motors: Final stop")
        set_servo_pulse(GROUP1_MOTOR1, STOP_PULSE)
        set_servo_pulse(GROUP1_MOTOR2, STOP_PULSE)
        set_servo_pulse(GROUP2_MOTOR1, STOP_PULSE)
        set_servo_pulse(GROUP2_MOTOR2, STOP_PULSE)

        # Test opposite group directions
        print("\nTesting opposite group directions")
        print("Group 1: Forward, Group 2: Reverse")
        set_servo_pulse(GROUP1_MOTOR1, FORWARD_PULSE)  # S0 forward
        set_servo_pulse(GROUP1_MOTOR2, FORWARD_PULSE)  # S2 forward
        set_servo_pulse(GROUP2_MOTOR1, REVERSE_PULSE)  # S1 reverse
        set_servo_pulse(GROUP2_MOTOR2, REVERSE_PULSE)  # S3 reverse
        time.sleep(3)

        # Final stop
        print("All motors: Final stop")
        set_servo_pulse(GROUP1_MOTOR1, STOP_PULSE)
        set_servo_pulse(GROUP1_MOTOR2, STOP_PULSE)
        set_servo_pulse(GROUP2_MOTOR1, STOP_PULSE)
        set_servo_pulse(GROUP2_MOTOR2, STOP_PULSE)

        print("\nTesting completed successfully")

except Exception as e:
    import sys
    print("Error occurred:", e)
    sys.print_exception(e)

    # Emergency stop all motors in case of error
    try:
        print("Emergency stop: All motors")
        set_servo_pulse(0, STOP_PULSE)  # S0
        set_servo_pulse(1, STOP_PULSE)  # S1
        set_servo_pulse(2, STOP_PULSE)  # S2
        set_servo_pulse(3, STOP_PULSE)  # S3
    except:
        print("Emergency stop failed")