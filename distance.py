import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
import math
import tf_transformations

class Distance(Node):
    def __init__(self):
        super().__init__('distance')
        # 激光雷达订阅
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
            
        # IMU订阅
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/mpu6050',  # 根据实际IMU话题名修改
            self.imu_callback,
            10)
            
        # 距离发布器
        self.distance_publisher = self.create_publisher(
            Float32MultiArray,
            'distance_data',
            10)
            
        # 初始化IMU姿态角
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
    def imu_callback(self, msg):
        # 从四元数获取欧拉角
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
    
    def is_angle_between(self, angle, start, end):
        """处理角度范围跨越-π/π边界的情况"""
        if start <= end:
            return start <= angle <= end
        else:  # 角度范围跨越-π/π边界
            return angle >= start or angle <= end
    
    def adjust_angle(self, angle):
        # 根据IMU的偏航角调整激光雷达角度
        adjusted_angle = angle - self.yaw
        # 确保角度在-π到π之间
        while adjusted_angle > math.pi:
            adjusted_angle -= 2 * math.pi
        while adjusted_angle < -math.pi:
            adjusted_angle += 2 * math.pi
        return adjusted_angle
    
    def lidar_callback(self, msg):
        # 获取激光雷达数据
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        num_ranges = len(ranges)

        # 计算左侧和后侧的角度范围索引(引入经由IMU调整后的角度)
        left_angle_start = self.adjust_angle(math.pi / 2 - math.pi / 4)
        left_angle_end = self.adjust_angle(math.pi / 2 + math.pi / 4)
        rear_angle_start = self.adjust_angle(math.pi - math.pi / 4)
        rear_angle_end = self.adjust_angle(math.pi + math.pi / 4)

        left_distances = []
        rear_distances = []


        for i in range(num_ranges):
            current_angle = self.adjust_angle(angle_min + i * angle_increment)
            
            # 处理左侧范围
            if self.is_angle_between(current_angle, left_angle_start, left_angle_end):
                if msg.range_min < ranges[i] < msg.range_max:
                    # 根据倾斜角度修正距离
                    corrected_distance = ranges[i] * math.cos(self.pitch) * math.cos(self.roll)
                    left_distances.append(corrected_distance)
                    
            # 处理后侧范围
            elif self.is_angle_between(current_angle, rear_angle_start, rear_angle_end):
                if msg.range_min < ranges[i] < msg.range_max:
                    # 根据倾斜角度修正距离
                    corrected_distance = ranges[i] * math.cos(self.pitch) * math.cos(self.roll)
                    rear_distances.append(corrected_distance)

        # 计算左侧和后侧的最小距离
        left_min_distance = self.get_min_distance(left_distances)
        rear_min_distance = self.get_min_distance(rear_distances)

        # 发布距离数据
        distance_msg = Float32MultiArray()
        distance_msg.data = [left_min_distance, rear_min_distance]
        self.distance_publisher.publish(distance_msg)

        # 输出结果
        self.get_logger().info(f'左侧最近距离: {left_min_distance:.2f} m')
        self.get_logger().info(f'后侧最近距离: {rear_min_distance:.2f} m')

    def get_min_distance(self, distances):
        if not distances:
            return -1.0  # 表示无有效数据
        return min(distances)
    
def main(args=None):
    rclpy.init(args=args)
    distance = Distance()
    rclpy.spin(distance)
    distance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
