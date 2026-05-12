#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty

class TopicMonitorNode(Node):
    def __init__(self):
        super().__init__('topic_monitor_node')
        
        # 訂閱 /rtabmap/odom_last_frame 話題
        self.subscription = self.create_subscription(
            String,
            '/rtabmap/odom_last_frame',
            self.topic_callback,
            10
        )

        # 創建服務客戶端
        self.service_client = self.create_client(Empty, '/rtabmap/rtabmap/set_mode_localization')
        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

        # 初始化變量
        self.last_msg_time = self.get_clock().now().to_msg().sec
        self.timeout = 2.0  # 定義超時時間（秒）
        self.service_called = False  # 避免重複調用服務
        self.last_received_value = None  # 存儲最近接收的話題值

        # 創建定時器，每秒檢查一次超時
        self.timer = self.create_timer(1.0, self.check_timeout)

    def topic_callback(self, msg):
        # 接收並顯示話題值
        self.last_received_value = msg.data
        self.get_logger().info(f'Received topic value: {self.last_received_value}')
        
        # 更新最後消息時間並重置服務調用標誌
        self.last_msg_time = self.get_clock().now().to_msg().sec
        self.service_called = False

    def check_timeout(self):
        # 檢查是否超過超時時間
        current_time = self.get_clock().now().to_msg().sec
        if (current_time - self.last_msg_time >= self.timeout) and not self.service_called:
            self.get_logger().info(f'No message received for {self.timeout} seconds, calling service...')
            self.get_logger().info(f'Last received topic value: {self.last_received_value if self.last_received_value is not None else "None"}')
            self.call_service()
            self.service_called = True

    def call_service(self):
        req = Empty.Request()
        future = self.service_client.call_async(req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Service called successfully!')
        except Exception as e:
            self.get_logger().info(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TopicMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()