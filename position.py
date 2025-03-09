import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
import math
import tf_transformations
import smbus2

# TODO: Waiting for testing

class PositionNode(Node):
    def __init__(self):
        super().__init__('position_node')
        
        # I2C初始化
        self.bus = smbus2.SMBus(1)
        self.slave_address = 0x54

         # 添加场地尺寸参数（单位：米）
        self.field_length = 8.0  # 场地长度
        self.field_width = 5.0   # 场地宽度
        
        # 添加位置估计
        self.position_x = 0.0    # X坐标（相对于场地左下角）
        self.position_y = 0.0    # Y坐标（相对于场地左下角）

        # 定义状态码
        self.STATUS_VALID = 0x01           # 数据有效
        self.STATUS_POSITION_ERROR = 0x02  # 位置超出范围
        self.STATUS_LEFT_ERROR = 0x04      # 左侧距离异常
        self.STATUS_REAR_ERROR = 0x08      # 后侧距离异常
        self.STATUS_IMU_ERROR = 0x10       # IMU数据异常

        # 初始化数据帧
        self.data = [
            0x55, 0xAA,  # 特征标头 (2字节)
            0x00, 0x00,  # 状态码 (2字节: 高字节保留，低字节为状态标志位)
            0x00, 0x00,  # X坐标 (2字节，单位：厘米)
            0x00, 0x00,  # Y坐标 (2字节，单位：厘米)
            0x00, 0x00,  # 左侧距离 (2字节，单位：厘米)
            0x00, 0x00,  # 后侧距离 (2字节，单位：厘米)
            0x00, 0x00,  # 航向角 (2字节，单位：0.01弧度)
            0x00, 0x00   # 校验和 (2字节)
        ]

        # 激光雷达订阅
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
            
        # IMU订阅
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/mpu6050',
            self.imu_callback,
            10)

        # 创建定时器，每隔10ms发送一次I2C数据
        self.timer = self.create_timer(0.01, self.send_i2c_data)

        # 初始化IMU姿态角
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # 初始化距离数据
        self.left_distance = -1.0
        self.rear_distance = -1.0

    def validate_distances(self, left_distance, rear_distance):
        """验证距离并更新状态码"""
        status = 0x00
        is_valid = True
        
        # 检查距离值是否有效
        if left_distance < 0:
            status |= self.STATUS_LEFT_ERROR
            is_valid = False
        if rear_distance < 0:
            status |= self.STATUS_REAR_ERROR
            is_valid = False

        # 计算预期的位置
        if is_valid:
            expected_x = self.field_width - left_distance
            expected_y = rear_distance
            
            # 检查是否在场地范围内
            if not (0 <= expected_x <= self.field_width):
                status |= self.STATUS_POSITION_ERROR
                self.get_logger().warn(f'X坐标超出范围: {expected_x:.2f}m')
                is_valid = False
                
            if not (0 <= expected_y <= self.field_length):
                status |= self.STATUS_POSITION_ERROR
                self.get_logger().warn(f'Y坐标超出范围: {expected_y:.2f}m')
                is_valid = False

        # 检查IMU数据
        if abs(self.roll) > math.pi/6 or abs(self.pitch) > math.pi/6:
            status |= self.STATUS_IMU_ERROR
            is_valid = False

        # 更新状态码
        if is_valid:
            status |= self.STATUS_VALID
            self.position_x = expected_x
            self.position_y = expected_y
            
            # 更新数据帧中的位置信息（转换为厘米）
            x_cm = int(self.position_x * 100)
            y_cm = int(self.position_y * 100)
            self.data[4] = (x_cm >> 8) & 0xFF
            self.data[5] = x_cm & 0xFF
            self.data[6] = (y_cm >> 8) & 0xFF
            self.data[7] = y_cm & 0xFF

        # 更新数据帧中的状态码
        self.data[2] = 0x00  # 高字节保留
        self.data[3] = status
        
        return is_valid

    def imu_callback(self, msg):
        quaternion = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        euler = tf_transformations.euler_from_quaternion(quaternion)
        self.roll = euler[0]
        self.pitch = euler[1]
        self.yaw = euler[2]

        # 更新航向角数据（转换为0.01弧度单位）
        yaw_fixed = int(self.yaw * 100)  # 转换为0.01弧度
        self.data[12] = (yaw_fixed >> 8) & 0xFF
        self.data[13] = yaw_fixed & 0xFF

    def is_angle_between(self, angle, start, end):
        if start <= end:
            return start <= angle <= end
        else:
            return angle >= start or angle <= end

    def adjust_angle(self, angle):
        adjusted_angle = angle - self.yaw
        while adjusted_angle > math.pi:
            adjusted_angle -= 2 * math.pi
        while adjusted_angle < -math.pi:
            adjusted_angle += 2 * math.pi
        return adjusted_angle

    def lidar_callback(self, msg):
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        num_ranges = len(ranges)

        left_angle_start = self.adjust_angle(math.pi / 2 - math.pi / 4)
        left_angle_end = self.adjust_angle(math.pi / 2 + math.pi / 4)
        rear_angle_start = self.adjust_angle(math.pi - math.pi / 4)
        rear_angle_end = self.adjust_angle(math.pi + math.pi / 4)

        left_distances = []
        rear_distances = []

        for i in range(num_ranges):
            current_angle = self.adjust_angle(angle_min + i * angle_increment)
            
            if self.is_angle_between(current_angle, left_angle_start, left_angle_end):
                if msg.range_min < ranges[i] < msg.range_max:
                    corrected_distance = ranges[i] * math.cos(self.pitch) * math.cos(self.roll)
                    left_distances.append(corrected_distance)
                    
            elif self.is_angle_between(current_angle, rear_angle_start, rear_angle_end):
                if msg.range_min < ranges[i] < msg.range_max:
                    corrected_distance = ranges[i] * math.cos(self.pitch) * math.cos(self.roll)
                    rear_distances.append(corrected_distance)

        # 更新距离值
        self.left_distance = self.get_min_distance(left_distances)
        self.rear_distance = self.get_min_distance(rear_distances)

        # 转换为厘米并保留2位小数
        left_distance_cm = max(0, self.left_distance * 100)  # 米转换为厘米
        rear_distance_cm = max(0, self.rear_distance * 100)  # 米转换为厘米
        
        # 将浮点数转换为定点数(保留2位小数)
        left_distance_fixed = int(left_distance_cm * 100)
        rear_distance_fixed = int(rear_distance_cm * 100)
        
        # 限制在16位范围内
        left_distance_fixed = min(65535, max(0, left_distance_fixed))
        rear_distance_fixed = min(65535, max(0, rear_distance_fixed))
        
        # 更新数据帧中的距离值
        self.data[8] = (left_distance_fixed >> 8) & 0xFF
        self.data[9] = left_distance_fixed & 0xFF
        self.data[10] = (rear_distance_fixed >> 8) & 0xFF
        self.data[11] = rear_distance_fixed & 0xFF

    def get_min_distance(self, distances):
        if not distances:
            return -1.0
        return min(distances)

    def send_i2c_data(self):
        try:
            # 数据长度检查
            if len(self.data) != 16:
                self.get_logger().error('数据长度错误: 必须为16字节')
                return

            # 验证数据并更新状态
            if self.validate_distances(self.left_distance, self.rear_distance):
                # 计算校验和
                checksum = sum(self.data[:-2]) & 0xFFFF
                self.data[14] = (checksum >> 8) & 0xFF
                self.data[15] = checksum & 0xFF

                # 输出状态信息
                status = self.data[3]
                status_str = []
                if status & self.STATUS_VALID:
                    status_str.append("数据有效")
                if status & self.STATUS_POSITION_ERROR:
                    status_str.append("位置超出范围")
                if status & self.STATUS_LEFT_ERROR:
                    status_str.append("左侧距离异常")
                if status & self.STATUS_REAR_ERROR:
                    status_str.append("后侧距离异常")
                if status & self.STATUS_IMU_ERROR:
                    status_str.append("IMU数据异常")

                self.get_logger().info(f'状态: {", ".join(status_str)}')
                self.get_logger().info(f'当前位置: X={self.position_x:.2f}m, Y={self.position_y:.2f}m')

                # 发送数据
                self.bus.write_i2c_block_data(self.slave_address, 0, self.data)
            else:
                self.get_logger().warn('数据无效，不发送')

        except Exception as e:
            self.get_logger().error(f'发送数据错误: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()