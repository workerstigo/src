#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

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

        # ── 新方法：監聽 /cmd_vel 判斷左右馬達是否靜止 ──────────────────
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info('Subscription created for /cmd_vel')

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
        self.odom_timeout   = 1.0      # 超過 1 秒沒收到點雲 → 切定位

        # ── 新方法參數（馬達靜止計時）────────────────────────────────────
        self.wheel_base  = 0.30        # 左右輪中心距離 (m)
        self.max_linear  = 0.20        # 最大線速度 (m/s)
        self.max_pwm     = 255
        self.stop_timeout = 10.0       # 馬達都為 0 超過此秒數 → 切定位

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
    # 舊方法：收到點雲 → 還在移動 → 確保建圖模式
    # ════════════════════════════════════════════════════════════════════
    def odom_callback(self, msg):
        self.last_odom_time = self.get_clock().now()
        if self.mode != "mapping":
            self.get_logger().info('[Old] odom_last_frame received → switching to MAPPING mode')
            self.call_service(self.srv_mapping)
            self.mode = "mapping"

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
        left_pwm, right_pwm = self._calc_pwm(msg.linear.x, msg.angular.z)
        motors_zero = (left_pwm == 0 and right_pwm == 0)

        if not motors_zero:
            # 馬達有動 → 更新時間戳
            self.last_moving_time = self.get_clock().now()

            if not self.motor_is_moving:
                self.get_logger().info(
                    f'[New] Motors started (L={left_pwm}, R={right_pwm})'
                )
                self.motor_is_moving = True

            # 如果是定位模式，切回建圖模式
            if self.mode != "mapping":
                self.get_logger().info(
                    f'[New] Motors moving (L={left_pwm}, R={right_pwm}) → switching to MAPPING mode'
                )
                self.call_service(self.srv_mapping)
                self.mode = "mapping"
        else:
            if self.motor_is_moving:
                self.get_logger().info('[New] Motors stopped (L=0, R=0). Idle timer started...')
                self.motor_is_moving = False

    # ════════════════════════════════════════════════════════════════════
    # 定時檢查：兩個條件都可觸發切換
    # ════════════════════════════════════════════════════════════════════
    def check_timeout(self):
        if self.mode == "localization":
            return  # 已在定位模式，不重複切換

        current_time = self.get_clock().now()

        # 舊方法：點雲長時間未收到
        odom_elapsed  = (current_time - self.last_odom_time ).nanoseconds * 1e-9
        # 新方法：馬達長時間為零
        motor_elapsed = (current_time - self.last_moving_time).nanoseconds * 1e-9

        trigger_odom  = odom_elapsed  >= self.odom_timeout
        trigger_motor = motor_elapsed >= self.stop_timeout

        if trigger_odom or trigger_motor:
            reasons = []
            if trigger_odom:
                reasons.append(f'no odom_last_frame for {odom_elapsed:.1f}s')
            if trigger_motor:
                reasons.append(f'motors zero for {motor_elapsed:.1f}s')

            self.get_logger().info(
                f'Switching to LOCALIZATION mode. Reason: {", ".join(reasons)}'
            )
            self.call_service(self.srv_localization)
            self.mode = "localization"
            self.get_logger().info('Reset odometry.')
            self.call_service(self.srv_reset_odom)

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