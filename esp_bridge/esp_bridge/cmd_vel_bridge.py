import json
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String
import websocket


class CmdVelBridgeNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        self.ws_url = 'ws://localhost:8080/esp32'
        self.enabled = False

        # Match the web UI and ESP32 tank-control assumptions.
        self.wheel_base = 0.20
        self.max_linear_speed = 0.50
        self.max_pwm = 255
        self.min_pwm = 60
        self.deadband_linear = 0.01
        self.deadband_angular = 0.01

        self.ws = None
        self.connect_ws()

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        self.toggle_sub = self.create_subscription(
            Bool,
            '/nav_toggle',
            self.toggle_callback,
            10)

        self.feedback_pub = self.create_publisher(String, '/esp32/feedback', 10)
        self.get_logger().info(
            f'CmdVelBridge started. Nav output is {"enabled" if self.enabled else "disabled"}')

    def toggle_callback(self, msg: Bool):
        old_enabled = self.enabled
        self.enabled = msg.data
        self.get_logger().info(
            f'Nav output {"enabled" if self.enabled else "disabled"}')

        if old_enabled and not self.enabled:
            self.send_stop_command()

    def send_stop_command(self):
        self.send_motor_command(0, 0)
        self.get_logger().info('Stop command sent')

    def connect_ws(self):
        def on_open(ws):
            self.get_logger().info('Connected to ESP32 proxy')

        def on_message(ws, message):
            try:
                data = json.loads(message)
                if data.get('type') not in ['ping', 'pong']:
                    msg = String()
                    msg.data = message
                    self.feedback_pub.publish(msg)
            except Exception:
                pass

        def on_error(ws, error):
            self.get_logger().error(f'WebSocket error: {error}')

        def on_close(ws, close_status_code, close_msg):
            self.get_logger().warn('WebSocket closed, reconnecting...')
            time.sleep(3)
            self.connect_ws()

        try:
            self.ws = websocket.WebSocketApp(
                self.ws_url,
                on_open=on_open,
                on_message=on_message,
                on_error=on_error,
                on_close=on_close)
            wst = threading.Thread(target=self.ws.run_forever, daemon=True)
            wst.start()
        except Exception as e:
            self.get_logger().error(f'WebSocket connection failed: {e}')
            time.sleep(5)
            self.connect_ws()

    def cmd_vel_callback(self, msg: Twist):
        if not self.enabled:
            return

        x = msg.linear.x
        z = msg.angular.z

        if abs(x) < self.deadband_linear:
            x = 0.0
        if abs(z) < self.deadband_angular:
            z = 0.0

        left_vel = x - (z * self.wheel_base / 2.0)
        right_vel = x + (z * self.wheel_base / 2.0)

        left_pwm = self.velocity_to_pwm(left_vel)
        right_pwm = self.velocity_to_pwm(right_vel)

        self.send_motor_command(left_pwm, right_pwm)

        if left_pwm != 0 or right_pwm != 0:
            self.get_logger().info(
                f'Nav diff cmd: vx={x:.3f}, wz={z:.3f} -> L={left_pwm}, R={right_pwm}')

    def velocity_to_pwm(self, velocity):
        pwm = int((velocity / self.max_linear_speed) * self.max_pwm)
        pwm = max(-self.max_pwm, min(self.max_pwm, pwm))

        if pwm != 0 and abs(pwm) < self.min_pwm:
            pwm = self.min_pwm if pwm > 0 else -self.min_pwm

        return pwm

    def send_motor_command(self, left_pwm, right_pwm):
        if self.ws is None or not self.ws.sock or not self.ws.sock.connected:
            return

        command = {
            'type': 'motor',
            'left': int(left_pwm),
            'right': int(right_pwm),
        }

        try:
            self.ws.send(json.dumps(command))
        except Exception as e:
            self.get_logger().warn(f'Failed to send motor command: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

