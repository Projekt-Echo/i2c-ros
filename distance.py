import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class Distance(Node):
    def __init__(self):
        super().__init__('distance')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.subscription
    
    def lidar_callback(self, msg):
        # 获取激光雷达数据
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        num_ranges = len(ranges)

        # 计算左侧和后侧的角度范围索引
        left_angle_start = math.pi / 2 - math.pi / 4  # 左侧起始角度（60度）
        left_angle_end = math.pi / 2 + math.pi / 4    # 左侧结束角度（120度）
        rear_angle_start = math.pi - math.pi / 4      # 后侧起始角度（135度）
        rear_angle_end = math.pi + math.pi / 4        # 后侧结束角度（225度）

        left_distances = []
        rear_distances = []

        for i in range(num_ranges):
            current_angle = angle_min + i * angle_increment
            # 判断当前角度是否在左侧范围内
            if left_angle_start <= current_angle <= left_angle_end:
                if ranges[i] > msg.range_min and ranges[i] < msg.range_max:
                    left_distances.append(ranges[i])
            # 判断当前角度是否在后侧范围内
            elif rear_angle_start <= current_angle <= rear_angle_end:
                if ranges[i] > msg.range_min and ranges[i] < msg.range_max:
                    rear_distances.append(ranges[i])

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
