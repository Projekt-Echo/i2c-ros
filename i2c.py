import rclpy
from rclpy.node import Node
import smbus2
import time

class I2CSender(Node):
    def __init__(self):
        super().__init__('position_sender')
        self.bus = smbus2.SMBus(1)  # 使用I2C-1接口
        self.slave_address = 0x54  # STM32的从机地址

        self.distance_subscription = self.create_subscription(
            Float32MultiArray,
            'distance_data',
            self.distance_callback,
            10)

        # 创建定时器，每隔0.01秒(10ms)发送一次数据
        self.timer = self.create_timer(0.01, self.send_data)

        # 初始化数据
        self.data = [
            0x55, 0x00,  # 特征标头 (2字节)
            0x00, 0x00,  # 状态码 (2字节)
            0x00, 0x00,  # X坐标 (2字节)
            0x00, 0x00,  # Y坐标 (2字节)
            0x00, 0x00,  # 左侧距离 (2字节)
            0x00, 0x00,  # 后侧距离 (2字节)
            0x00, 0x00,  # 预留 (2字节)
            0x00, 0x00   # 预留 (2字节)
        ]

    def distance_callback(self, msg):
        left_distance = msg.data[0] * 100  # 转换为厘米
        rear_distance = msg.data[1] * 100  # 转换为厘米
        
        # 确保数值在单字节范围内 (0-255)
        left_distance = min(255, max(0, left_distance))
        rear_distance = min(255, max(0, rear_distance))
        
        # 更新data3和data4
        self.data[3] = left_distance
        self.data[4] = rear_distance


    def send_data(self):
        try:

            # 数据长度检查
            if len(self.data) != 16:
                self.get_logger().error('数据长度错误: 必须为16字节')
                return
        # 发送数据前的日志输出
            # 将数据转换回实际距离值进行显示
            left_value = ((self.data[8] << 8) | self.data[9]) / 100.0
            rear_value = ((self.data[10] << 8) | self.data[11]) / 100.0

            self.get_logger().info(f'发送数据:')
            self.get_logger().info(f'特征标头: 0x{self.data[0]:02X}{self.data[1]:02X}')
            self.get_logger().info(f'左侧距离: {left_value:.2f} cm')
            self.get_logger().info(f'后侧距离: {rear_value:.2f} cm')

            # 发送数据
            self.bus.write_i2c_block_data(self.slave_address, 0, data)
        except Exception as e:
            self.get_logger().error(f'发送数据错误: {e}')

def main(args=None):
    rclpy.init(args=args)
    i2c_sender = I2CSender()
    rclpy.spin(i2c_sender)
    i2c_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()