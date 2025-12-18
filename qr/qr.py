import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pyzbar.pyzbar import decode
import numpy as np

class QRCodeReader(Node):
    def __init__(self):
        super().__init__('qr_code_reader')

        # パラメータ宣言・取得
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('output_topic', '/qr_code_image')

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10)

        self.publisher_ = self.create_publisher(Image, output_topic, 10)

        self.bridge = CvBridge()

    def image_callback(self, msg):
        # ROS画像メッセージをOpenCV画像に変換
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 必要に応じて拡大して精度を上げる（例: 1.5倍）
        scale = 1.5
        scaled_frame = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)

        # QRコード検出
        qr_codes = decode(scaled_frame)

        for qr in qr_codes:
            data = qr.data.decode('utf-8')
            self.get_logger().info(f'QR Code detected: {data}')

            # ポリゴン取得
            points = qr.polygon
            if len(points) > 4:
                hull = cv2.convexHull(
                    np.array([(point.x, point.y) for point in points], dtype=np.float32)
                )
                hull = list(map(tuple, np.squeeze(hull)))
            else:
                hull = [(point.x, point.y) for point in points]

            # ポリゴンを元のスケールに戻す
            hull = [(int(x / scale), int(y / scale)) for (x, y) in hull]

            # 緑の枠線を描画
            n = len(hull)
            for j in range(n):
                cv2.line(frame, hull[j], hull[(j + 1) % n], (0, 255, 0), 2)

            # テキスト表示
            x, y = hull[0][0], hull[0][1] - 10
            cv2.putText(frame, data, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255), 2)

        # 結果をROSトピックにパブリッシュ
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
