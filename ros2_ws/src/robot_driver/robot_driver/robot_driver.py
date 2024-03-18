import rclpy
import serial
from rclpy.node import Node
from math import sin, cos, pi

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry

import RPi.GPIO as GPIO


class Motor():
    def __init__(self, enable, i1, i2):
        self.enable = enable
        self.i1 = i1
        self.i2 = i2
        self.velocity = 0
        self.target = 0
        self.error = 0
        self.ierror = 0
        self.wheel_radius = 0.06
        self.ticks = 0
        # Pins
        GPIO.setup(self.enable, GPIO.OUT)
        GPIO.setup(self.i1, GPIO.OUT)
        GPIO.setup(self.i2, GPIO.OUT)

        # PWM
        self.pwm = GPIO.PWM(self.enable, 100)
        self.pwm.start(0)

    def pid(self, elapsed, ticks):
        kp = 1.5
        ki = 4.0
        kd = 0.0
        meters_per_tick = (pi*self.wheel_radius) / (30*100)

        difference = (ticks % 4096) - (self.ticks % 4096)
        if (difference > 2048):
            difference -= 4096
        elif (difference < -2048):
            difference += 4096

        self.velocity = difference * meters_per_tick / elapsed
        self.ticks = ticks

        olderror = self.error
        self.error = self.velocity - self.target
        self.ierror += self.error * elapsed
        derror = (self.error - olderror) / elapsed
        pwm = kp * self.error + ki * self.ierror + kd * derror

        self.run(pwm)

    def run(self, pwm):
        i1 = GPIO.LOW
        i2 = GPIO.LOW
        if pwm > 0:
            i1 = GPIO.HIGH
            i2 = GPIO.LOW
        else:
            i1 = GPIO.LOW
            i2 = GPIO.HIGH

        GPIO.output(self.i1, i1)
        GPIO.output(self.i2, i2)
        self.pwm.ChangeDutyCycle(max(min(abs(100*pwm), 100), 0))


class RobotDriver(Node):
    def __init__(self):
        super().__init__('robot_driver')

        self.odom_publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.vel_publisher_ = self.create_publisher(Twist, 'vel', 10)
        self.odom_broadcaster_ = TransformBroadcaster(self)
        self.cmd_vel_subscription_ = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.timer = self.create_timer(1/100, self.update)
        self.width = 0.275
        self.then = self.get_clock().now()
        self.leftMotor = Motor(13, 26, 19)
        self.rightMotor = Motor(12, 16, 20)
        self.x = 0
        self.y = 0
        self.th = 0
        self.vx = 0
        self.vth = 0
        self.halt = False
        self.serial = serial.Serial('/dev/serial/by-id/usb-ATMEL_mEDBG_CMSIS-DAP_8BF897446D846781B338-if01', 9600)

    def cmd_vel_callback(self, msg):
        if(self.halt):
            self.leftMotor.target = 0
            self.rightMotor.target = 0
        else:
            linear = max(min(msg.linear.x, 0.25), -0.25)
            delta = max(min(msg.angular.z, 0.5), -0.5) * self.width / 2
            
            self.leftMotor.target = linear - delta
            self.rightMotor.target = linear + delta

    def update(self):
        elapsed, now = self.cycle_setup()
        # Read Serial
        line = self.serial.readline().decode('utf-8').strip()
        str_val1, str_val2, encoder_L, encoder_R = line.split(',')
        val1 = int(str_val1)
        val2 = int(str_val2)
        #if(val1>400 or val2>400):
        #    self.halt = True
        #    self.leftMotor.target = 0
        #    self.rightMotor.target = 0
        #else:
        #    self.halt = False
        left_ticks, right_ticks = int(encoder_L), int(encoder_R)
        self.get_logger().info(f'\nleft ticks:  {left_ticks}\nright_ticks: {right_ticks}\n')

        # PID
        self.leftMotor.pid(elapsed, left_ticks)
        self.rightMotor.pid(elapsed, right_ticks)

        # ODOMETRY
        self.calculate_odometry(elapsed)

        # PUBLISHING
        self.publish_odometry(now)

        # Create twist
        twist = Twist()
        twist.linear.x = (self.leftMotor.velocity + self.rightMotor.velocity) / 2
        twist.angular.z = (self.leftMotor.velocity - self.rightMotor.velocity) / self.width
        self.vel_publisher_.publish(twist)

    def cycle_setup(self):
        now = self.get_clock().now()
        elapsed = now - self.then
        elapsed = elapsed.nanoseconds / 1000000000
        self.then = now
        self.get_logger().info(f'Running at {1/elapsed} Hz\nLeft Velocity: {self.leftMotor.velocity}\nRight Velocity: {self.rightMotor.velocity}')
        return elapsed, now

    def publish_odometry(self, now):
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = -sin(self.th / 2)
        quaternion.w = cos(self.th / 2)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.twist.twist.linear.x = -self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vth
        self.odom_publisher_.publish(odom)

        odom_tf = TransformStamped()
        odom_tf.header.stamp = now.to_msg()
        odom_tf.header.frame_id = 'odom'
        odom_tf.child_frame_id = 'base_link'
        odom_tf.transform.translation.x = self.x
        odom_tf.transform.translation.y = self.y
        odom_tf.transform.translation.z = 0.0
        odom_tf.transform.rotation = quaternion
        self.odom_broadcaster_.sendTransform(odom_tf)

    def calculate_odometry(self, elapsed):
        self.vx = (self.leftMotor.velocity + self.rightMotor.velocity) / 2
        self.vth = (self.leftMotor.velocity - self.rightMotor.velocity) / self.width

        self.th += self.vth * elapsed
        self.x += cos(self.th) * self.vx * elapsed
        self.y += -sin(self.th) * self.vx * elapsed


def main(args=None):
    rclpy.init(args=args)

    GPIO.setmode(GPIO.BCM)
    node = RobotDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()


if __name__ == '__main__':
    main()
