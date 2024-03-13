import rclpy
from rclpy.node import Node
from math import sin, cos, pi

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry

import RPi.GPIO as GPIO


class Motor():
    def __init__(self, enable, i1, i2, encoder_a, encoder_b):
        self.enable = enable
        self.i1 = i1
        self.i2 = i2
        self.encoder_a = encoder_a
        self.encoder_b = encoder_b
        self.ticks = 0
        self.velocity = 0
        self.target = 0
        self.error = 0
        self.ierror = 0
        self.wheel_radius = 0.06
        # Pins
        GPIO.setup(self.enable, GPIO.OUT)
        GPIO.setup(self.i1, GPIO.OUT)
        GPIO.setup(self.i2, GPIO.OUT)
        GPIO.setup(self.encoder_a, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.encoder_b, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        # PWM
        self.pwm = GPIO.PWM(self.enable, 60)
        self.pwm.start(0)

        # Interrupt
        GPIO.add_event_detect(self.encoder_a, GPIO.RISING,
                              callback=self.encoder_interrupt)

    def pid(self, elapsed):
        kp = 1.00
        ki = 0.10
        kd = 0.01
        meters_per_tick = (2*pi*self.wheel_radius) / (30*100)
        self.velocity = self.ticks * meters_per_tick / elapsed

        olderror = self.error
        self.error = self.velocity - self.target
        self.ierror += self.error * elapsed
        derror = (self.error - olderror) / elapsed
        pwm = kp * self.error + ki * self.ierror + kd * derror
        self.ticks = 0

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

    def encoder_interrupt(self, _pin):
        if (GPIO.input(self.encoder_b)):
            self.ticks += 1
        else:
            self.ticks -= 1


class RobotDriver(Node):
    def __init__(self):
        super().__init__('robot_driver')
        self.odom_publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.odom_broadcaster_ = TransformBroadcaster(self)
        self.cmd_vel_subscription_ = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.timer = self.create_timer(0.01, self.update)
        self.width = 0.275
        self.then = self.get_clock().now()
        self.leftMotor = Motor(13, 26, 19, 17, 27)
        self.rightMotor = Motor(12, 16, 20, 23, 24)
        self.x = 0
        self.y = 0
        self.th = 0
        self.vx = 0
        self.vth = 0

    def cmd_vel_callback(self, msg):
        delta = msg.angular.z * self.width
        self.leftMotor.target = msg.linear.x - delta
        self.rightMotor.target = msg.linear.x + delta

    def update(self):
        elapsed, now = self.cycle_setup()

        # PID
        self.leftMotor.pid(elapsed)
        self.rightMotor.pid(elapsed)

        # ODOMETRY
        self.calculate_odometry(elapsed)

        # PUBLISHING
        self.publish_odometry(now)

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
        quaternion.z = sin(self.th / 2)
        quaternion.w = cos(self.th / 2)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.twist.twist.linear.x = self.vx
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
