#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from std_msgs.msg import String

from rclpy.qos import qos_profile_sensor_data


class TopicMonitorNode(Node):
    def __init__(self):
        super().__init__('topic_monitor_node')

        # ── 舊方法：監聽 /rtabmap/odom_last_frame (PointCloud2) ──────────
        self.sub_odom = self.create_subscription(
            PointCloud2,
            '/rtabmap/odom_last_frame',
            self.odom_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info('Subscription created for /rtabmap/odom_last_frame')

        # ── 新方法：監聯 /cmd_vel 判斷左右馬達是否靜止 ─────────────────────
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info('Subscription created for /cmd_vel')

        # ── 新增：監聽 /esp32/feedback 使用硬體回傳判斷手動控制 ───────────────
        self.sub_feedback = self.create_subscription(
            String,
            '/esp32/feedback',
            self.feedback_callback,
            10
        )
        self.get_logger().info('Subscription created for /esp32/feedback')

        # ── RTAB-Map 服務 ─────────────────────────────────────────────────
        self.srv_reset_odom  = self.create_client(Empty, '/rtabmap/reset_odom')
        self.srv_localization = self.create_client(Empty, '/rtabmap/rtabmap/set_mode_localization')
        self.srv_mapping      = self.create_client(Empty, '/rtabmap/rtabmap/set_mode_mapping')

        while not (self.srv_localization.wait_for_service(timeout_sec=1.0) and
                   self.srv_mapping.wait_for_service(timeout_sec=1.0) and
                   self.srv_reset_odom.wait_for_service(timeout_sec=1.0)):
            self.get_logger().info('Waiting for RTAB-Map services...')

        # ── 舊方法參數 ────────────────────────────────────────────────────
        # 最後一次收到 odom_last_frame 的時間
        self.last_odom_time = self.get_clock().now()
        self.odom_timeout   = 10.0     # 超過 10 秒沒收到點雲 → 切定位（原本 2 秒太短）

        # ── 新方法參數（馬達靜止計時）────────────────────────────────────
        self.wheel_base  = 0.30        # 左右輪中心距離 (m)
        self.max_linear  = 0.20        # 最大線速度 (m/s)
        self.max_pwm     = 255
        self.stop_timeout = 60.0       # 馬達都為 0 超過此秒數 → 切定位

        self.last_moving_time = self.get_clock().now()   # 最後一次馬達非零的時間
        self.motor_is_moving  = False  # 目前馬達是否有動

        # ── 共用狀態 ──────────────────────────────────────────────────────
        self.mode = "mapping"
        self.timer = self.create_timer(1.0, self.check_timeout)

        self.get_logger().info(
            f'Node started. '
            f'[Old] odom_last_frame timeout={self.odom_timeout}s  '
            f'[New] motor stop timeout={self.stop_timeout}s'
        )

    # ════════════════════════════════════════════════════════════════════
    # 舊方法：收到點雲 → 只更新時間戳，不再自動切換模式
    # （修正：原本會在 LOCALIZATION 時收到 odom 就切回 MAPPING，
    #   但 reset_odom 本身就會產生新 odom，造成無限循環）
    # ════════════════════════════════════════════════════════════════════
    def odom_callback(self, msg):
        self.last_odom_time = self.get_clock().now()

    # ════════════════════════════════════════════════════════════════════
    # 新方法：cmd_vel 馬達監聽
    # ════════════════════════════════════════════════════════════════════
    def _calc_pwm(self, linear_x: float, angular_z: float):
        left_vel  = linear_x - (angular_z * self.wheel_base / 2.0)
        right_vel = linear_x + (angular_z * self.wheel_base / 2.0)
        left_pwm  = int((left_vel  / self.max_linear) * self.max_pwm)
        right_pwm = int((right_vel / self.max_linear) * self.max_pwm)
        left_pwm  = max(min(left_pwm,  self.max_pwm), -self.max_pwm)
        right_pwm = max(min(right_pwm, self.max_pwm), -self.max_pwm)
        return left_pwm, right_pwm

    def cmd_vel_callback(self, msg: Twist):
        # 只要線速度或角速度絕對值大於 0.001 就視為正在移動
        is_moving = abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001

        if is_moving:
            # 馬達有動 → 更新時間戳
            self.last_moving_time = self.get_clock().now()

            if not self.motor_is_moving:
                self.get_logger().info(
                    f'[New] Movement detected (vx={msg.linear.x:.3f}, wz={msg.angular.z:.3f})'
                )
                self.motor_is_moving = True

            # 如果是定位模式，切回建圖模式
            if self.mode != "mapping":
                self.get_logger().info(
                    f'[New] Request from command → switching to MAPPING mode'
                )
                self.call_service(self.srv_mapping)
                self.mode = "mapping"
        else:
            if self.motor_is_moving:
                self.get_logger().info('[New] Motors idle. Idle timer started...')
                self.motor_is_moving = False

    def feedback_callback(self, msg: String):
        # 收到硬體回傳訊息，視為有人在操作
        self.last_moving_time = self.get_clock().now()
        
        if not self.motor_is_moving:
            self.get_logger().info(f'[Hardware] Received ESP32 feedback: {msg.data}')
            self.motor_is_moving = True

        if self.mode != "mapping":
            self.get_logger().info('[Hardware] Feedback from ESP32 → switching to MAPPING mode')
            self.call_service(self.srv_mapping)
            self.mode = "mapping"

    # ════════════════════════════════════════════════════════════════════
    # 定時檢查：只用馬達靜止條件觸發切換
    # （修正：移除 odom 觸發條件，避免與 odom_callback 產生循環）
    # （修正：不再重置 odometry，避免 TF 斷裂影響 Nav2）
    # ════════════════════════════════════════════════════════════════════
    def check_timeout(self):
        if self.mode == "localization":
            return  # 已在定位模式，不重複切換

        current_time = self.get_clock().now()

        # 只用馬達靜止時間作為切換條件
        motor_elapsed = (current_time - self.last_moving_time).nanoseconds * 1e-9

        if motor_elapsed >= self.stop_timeout:
            self.get_logger().info(
                f'Switching to LOCALIZATION mode. Reason: motors zero for {motor_elapsed:.1f}s'
            )
            self.call_service(self.srv_localization)
            self.mode = "localization"
            # 注意：不再重置 odometry，因為這會破壞 TF 連續性，導致 Nav2 無法工作

    # ════════════════════════════════════════════════════════════════════
    # 服務呼叫
    # ════════════════════════════════════════════════════════════════════
    def call_service(self, client):
        req = Empty.Request()
        future = client.call_async(req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            _ = future.result()
            self.get_logger().info('Service called successfully!')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


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