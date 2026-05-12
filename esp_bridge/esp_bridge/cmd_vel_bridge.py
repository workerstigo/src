import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
import websocket
import json
import threading
import time

class CmdVelBridgeNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # ======================== 參數設定 ========================
        self.ws_url = "ws://localhost:8080/esp32"
        self.enabled = False  # 預設不發送，由前端控制
        
        # WebSocket 連線
        self.ws = None
        self.connect_ws()

        # 訂閱 /cmd_vel (導航指令)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        # 訂閱 /nav_toggle (前端開關連動)
        self.toggle_sub = self.create_subscription(
            Bool,
            '/nav_toggle',
            self.toggle_callback,
            10)

        # 回傳 /esp32/feedback
        self.feedback_pub = self.create_publisher(String, '/esp32/feedback', 10)

        self.get_logger().info(f'CmdVelBridge 已啟動。狀態: {"開啟" if self.enabled else "關閉"}')

    def toggle_callback(self, msg: Bool):
        old_enabled = self.enabled
        self.enabled = msg.data
        status = "開啟" if self.enabled else "關閉"
        self.get_logger().info(f"導航發送功能已 {status}")
        
        # 如果從開啟轉為關閉，立刻補送一個停止指令
        if old_enabled and not self.enabled:
            self.send_stop_command()

    def send_stop_command(self):
        if self.ws and self.ws.sock and self.ws.sock.connected:
            stop_cmd = {
                "type": "joystick",
                "direction": "stop",
                "intensity": 0.0
            }
            try:
                self.ws.send(json.dumps(stop_cmd))
                self.get_logger().info("已發送強制停止指令 (Nav Disabled)")
            except:
                pass

    def connect_ws(self):
        def on_open(ws):
            self.get_logger().info("WebSocket 已連線到 Proxy!")

        def on_message(ws, message):
            try:
                data = json.loads(message)
                if data.get('type') not in ['ping', 'pong']:
                    msg = String()
                    msg.data = message
                    self.feedback_pub.publish(msg)
            except:
                pass

        def on_error(ws, error):
            self.get_logger().error(f"WebSocket 錯誤: {error}")

        def on_close(ws, close_status_code, close_msg):
            self.get_logger().warn("WebSocket 連線已中斷。3秒後重連...")
            time.sleep(3)
            self.connect_ws()

        try:
            self.ws = websocket.WebSocketApp(
                self.ws_url,
                on_open=on_open,
                on_message=on_message,
                on_error=on_error,
                on_close=on_close
            )
            wst = threading.Thread(target=self.ws.run_forever, daemon=True)
            wst.start()
        except Exception as e:
            self.get_logger().error(f"WebSocket 連線失敗: {e}")
            time.sleep(5)
            self.connect_ws()

    def cmd_vel_callback(self, msg: Twist):
        # 1. 檢查開關與連線
        if not self.enabled:
            return
            
        if self.ws is None or not self.ws.sock or not self.ws.sock.connected:
            return

        x = msg.linear.x
        z = msg.angular.z

        # 2. 將導航運動指令映射至 ESP32 聽得懂的「搖桿格式」 (暫時方案，解決馬達不動問題)
        # 邏輯：判斷主要動作方向
        direction = "stop"
        intensity = 0.0

        if abs(x) < 0.01 and abs(z) < 0.01:
            direction = "stop"
            intensity = 0.0
        elif abs(x) >= abs(z):
            # 以進退為主
            direction = "forward" if x > 0 else "backward"
            intensity = min(1.0, abs(x) / 0.5) # 映射 0.5m/s 為全速
        else:
            # 以轉向為主
            direction = "left" if z > 0 else "right"
            intensity = min(1.0, abs(z) / 1.0) # 映射 1.0rad/s 為全速

        # 3. 發送 JSON
        command = {
            "type": "joystick",
            "direction": direction,
            "intensity": round(intensity, 2)
        }

        try:
            self.ws.send(json.dumps(command))
            # 只有在非停止狀態才印日誌，避免刷頻
            if direction != "stop":
                self.get_logger().info(f"導航指令 (相容模式): {direction} | 強度: {intensity}")
        except Exception as e:
            pass

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
