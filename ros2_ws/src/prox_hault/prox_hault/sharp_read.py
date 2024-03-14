import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HaultPublisher(Node):

    def __init__(self):
        super().__init__('hault_Publisher')
        self.publisher_ = self.create_publisher((String), 'halt', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.hault = 0
        self.ser = serial.Serial('/dev/serial/by-id/usb-ATMEL_mEDBG_CMSIS-DAP_8BF897446D846781B338-if01', 9600)

    def timer_callback(self):
        line = self.ser.readline().decode('utf-8').strip()
        str_val1, str_val2 = line.split(',')
        val1 = int(str_val1)
        val2 = int(str_val2)
        msg = String()
        msg.data =str_val1
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        if(val1>400 or val2>400):
            self.hault = "not okay"
            msg.data = self.hault
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
        else:
            self.hault = "okay"
            msg.data = self.hault
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    hault_publisher = HaultPublisher()

    rclpy.spin(hault_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# ser = serial.Serial('COM3', 9600)
# while True:
#     line = ser.readline().decode('utf-8').strip()
#     str_val1, str_val2 = line.split(',')
#     val1 = int(str_val1)
#     val2 = int(str_val2)
#     if(val1>400 or val2>400):
#         hault =1
#         print(hault)
#     else:
#         hault =0
#         print(hault)