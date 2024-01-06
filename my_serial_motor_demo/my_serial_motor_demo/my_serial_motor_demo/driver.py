from my_serial_motor_demo_msgs.msg import MotorCommand
from my_serial_motor_demo_msgs.msg import MotorVels
from my_serial_motor_demo_msgs.msg import StepVals
import rclpy
from rclpy.node import Node

import serial
import struct
import time

class StepperMotorDriver(Node):

    def __init__(self):
        super().__init__('stepper_motor_driver')

        self.serial_port = '/dev/ttyACM0'  # Replace with your Arduino serial port
        self.baud_rate = 9600  # Set according to your Arduino code

        self.serial = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        time.sleep(2)  # Allow some time for the serial connection to establish

        self.subscription_motor_command = self.create_subscription(
            MotorCommand,
            'motor_command',
            self.motor_command_callback,
            10)

        self.publisher_motor_vels = self.create_publisher(MotorVels, 'motor_vels', 10)
        self.publisher_step_vals = self.create_publisher(StepVals, 'step_vals', 10)

    def motor_command_callback(self, motor_command):
        steps = motor_command.steps_m1
        steps1 = motor_command.steps_m2
        # Pack data using struct before sending
        packed_data = struct.pack('<ii', steps, steps1)  # Packing two integers into a byte array
        print(packed_data)
        self.serial.write(packed_data)

    # Add functions for publishing MotorVels and StepVals based on data received from Arduino

def main(args=None):
    rclpy.init(args=args)
    stepper_motor_driver = StepperMotorDriver()
    rclpy.spin(stepper_motor_driver)
    stepper_motor_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
