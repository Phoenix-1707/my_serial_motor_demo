from my_serial_motor_demo_msgs.msg import MotorCommand
from my_serial_motor_demo_msgs.msg import MotorVels
from my_serial_motor_demo_msgs.msg import StepVals

import rclpy
from rclpy.node import Node
from tkinter import *

class StepperMotorGUI(Node):

    def __init__(self):
        super().__init__('stepper_motor_gui')

        self.publisher = self.create_publisher(MotorCommand, 'motor_command', 10)
        self.speed_sub = self.create_subscription(MotorVels, 'motor_vels', self.motor_vel_callback, 10)
        self.encoder_sub = self.create_subscription(StepVals, 'step_vals', self.step_val_callback, 10)

        self.root = Tk()
        self.root.title("Stepper Motor GUI")

        self.motor1_steps_slider = Scale(self.root, from_=-1000, to=1000, orient=HORIZONTAL, length=300)
        self.motor1_steps_slider.pack()

        self.motor1_steps_entry = Entry(self.root)
        self.motor1_steps_entry.pack()

        self.motor2_steps_slider = Scale(self.root, from_=-1000, to=1000, orient=HORIZONTAL, length=300)
        self.motor2_steps_slider.pack()

        self.motor2_steps_entry = Entry(self.root)
        self.motor2_steps_entry.pack()

        self.update_button = Button(self.root, text="Update", command=self.send_steps_update)
        self.update_button.pack()

        self.send_once_button = Button(self.root, text="Send Once", command=self.send_once)
        self.send_once_button.pack()

        self.stop_motor_button = Button(self.root, text="Stop Motor", command=self.stop_motor)
        self.stop_motor_button.pack()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close) # Handle window closing

        self.is_closed = False # Flag to track if the window is closed

        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            self.is_closed = True

    def send_motor_command(self, motor1_steps, motor2_steps):
        msg = MotorCommand()
        msg.steps_m1 = motor1_steps
        msg.steps_m2 = motor2_steps
        self.publisher.publish(msg)

    def send_once(self):
        motor1_steps = self.motor1_steps_slider.get()
        motor2_steps = self.motor2_steps_slider.get()
        self.send_motor_command(motor1_steps, motor2_steps)

    def stop_motor(self):
        self.send_motor_command(0, 0) # Stop both motors

    def send_steps_update(self):
        motor1_steps_entry_text = self.motor1_steps_entry.get()
        motor2_steps_entry_text = self.motor2_steps_entry.get()

        if motor1_steps_entry_text and motor2_steps_entry_text:
            try:
                motor1_steps = int(motor1_steps_entry_text)
                motor2_steps = int(motor2_steps_entry_text)
                self.send_motor_command(motor1_steps, motor2_steps)
            except ValueError:
                print("Invalid input. Please enter a valid number.")


    def motor_vel_callback(self, motor_vels):
        # Update GUI with motor velocities
        pass

    def step_val_callback(self, step_vals):
        # Update GUI with step values
        pass

    def on_close(self):
        self.is_closed = True
        self.root.destroy()

def main(args=None):
    rclpy.init(args=args)
    stepper_motor_gui = StepperMotorGUI()
    while rclpy.ok() and not stepper_motor_gui.is_closed:
        rclpy.spin_once(stepper_motor_gui)
        stepper_motor_gui.root.update() # Update the GUI

    stepper_motor_gui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

'''
from my_serial_motor_demo_msgs.msg import MotorCommand
from my_serial_motor_demo_msgs.msg import MotorVels
from my_serial_motor_demo_msgs.msg import StepVals

import rclpy
from rclpy.node import Node
from tkinter import *

class StepperMotorGUI(Node):

    def __init__(self):
        super().__init__('stepper_motor_gui')

        self.publisher = self.create_publisher(MotorCommand, 'motor_command', 10)
        self.speed_sub = self.create_subscription(MotorVels, 'motor_vels', self.motor_vel_callback, 10)
        self.encoder_sub = self.create_subscription(StepVals, 'step_vals', self.step_val_callback, 10)

        self.root = Tk()
        self.root.title("Stepper Motor GUI")

        self.steps_slider = Scale(self.root, from_=0, to=1000, orient=HORIZONTAL, length=300)
        self.steps_slider.pack()

        self.steps_entry = Entry(self.root)
        self.steps_entry.pack()

        self.update_button = Button(self.root, text="Update", command=self.send_steps_update)
        self.update_button.pack()

        self.send_once_button = Button(self.root, text="Send Once", command=self.send_once)
        self.send_once_button.pack()

        self.stop_motor_button = Button(self.root, text="Stop Motor", command=self.stop_motor)
        self.stop_motor_button.pack()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)  # Handle window closing

        self.is_closed = False  # Flag to track if the window is closed

        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            self.is_closed = True

    def send_motor_command(self, steps):
        msg = MotorCommand()
        msg.steps_m1 = steps
        self.publisher.publish(msg)

    def send_once(self):
        steps = self.steps_slider.get()
        self.send_motor_command(steps)

    def stop_motor(self):
        self.send_motor_command(0)  # Stop the motor

    def send_steps_update(self):
        steps_entry_text = self.steps_entry.get()
        if steps_entry_text:
            try:
                steps = int(steps_entry_text)
                self.send_motor_command(steps)
            except ValueError:
                print("Invalid input. Please enter a valid number.")


    def motor_vel_callback(self, motor_vels):
        # Update GUI with motor velocities
        pass

    def step_val_callback(self, step_vals):
        # Update GUI with step values
        pass

    def on_close(self):
        self.is_closed = True
        self.root.destroy()

def main(args=None):
    rclpy.init(args=args)
    stepper_motor_gui = StepperMotorGUI()
    while rclpy.ok() and not stepper_motor_gui.is_closed:
        rclpy.spin_once(stepper_motor_gui)
        stepper_motor_gui.root.update()  # Update the GUI

    stepper_motor_gui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()'''