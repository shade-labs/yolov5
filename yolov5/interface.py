import torch
import requests
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, ObjectHypothesis, BoundingBox2D, Point2D, Pose2D
from geometry_msgs.msg import PoseWithCovariance, Point

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('yolov5_node')

        self.declare_parameter('pub_image', False)
        self.declare_parameter('pub_json', False)
        self.declare_parameter('pub_boxes', True)
        self.declare_parameter('weights_url', 'https://github.com/ultralytics/yolov5/releases/download/v6.1/yolov5m.pt')

        self.subscription = self.create_subscription(
            Image,
            'yolov5/image_raw',
            self.listener_callback,
            1
        )

        self.image_publisher = self.create_publisher(Image, 'yolov5/image', 10)
        self.json_publisher = self.create_publisher(String, 'yolov5/json', 10)
        self.detection_publisher = self.create_publisher(Detection2DArray, 'yolov5/detection_boxes', 10)

        self.counter = 0
        self.br = CvBridge()

        response = requests.get(self.get_parameter('weights_url').value)
        open("data.pt", "wb").write(response.content)
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='./data.pt')

        self.get_logger().info("YOLOv5 Initialized")

    def inference(self, image: Image):
        return self.model(image)

    def getDetectionArray(self, df):
        dda = Detection2DArray()

        detections = []
        self.counter += 1

        for row in df.itertuples():
            self.get_logger().info(f"Detected {row.name}")

            detection = Detection2D()

            detection.header.stamp = self.get_clock().now().to_msg()
            detection.header.frame_id = str(self.counter)

            hypothesises = []
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(row[6])
            hypothesis.hypothesis.score = float(row[5])

            pwc = PoseWithCovariance()
            pwc.pose.position = Point()
            pwc.pose.position.x = (int(row.xmin) + int(row.xmax)) / 2
            pwc.pose.position.y = (int(row.ymin) + int(row.ymax)) / 2

            hypothesis.pose = pwc

            hypothesises.append(hypothesis)
            detection.results = hypothesises

            bbox = BoundingBox2D()
            bbox.size_x = (int(row.xmax) - int(row.xmin)) / 2
            bbox.size_y = (int(row.ymax) - int(row.ymin)) / 2

            point = Point2D()
            point.x = (int(row.xmin) + int(row.xmax)) / 2
            point.y = (int(row.ymin) + int(row.ymax)) / 2

            center = Pose2D()
            center.position = point
            center.theta = 0.0

            detection.bbox = bbox

            detections.append(detection)

        dda.detections = detections
        dda.header.stamp = self.get_clock().now().to_msg()
        dda.header.frame_id = str(self.counter)
        return dda

    def listener_callback(self, data):
        self.get_logger().info('Got Image')
        current_frame = self.br.imgmsg_to_cv2(data)
        results = self.inference(current_frame)

        if self.get_parameter('pub_image').value:
            processed_image = self.br.cv2_to_imgmsg(results.imgs[0])
            self.image_publisher.publish(processed_image)

        if self.get_parameter('pub_json').value:
            json = String()
            json.data = results.pandas().xyxy[0].to_json(orient="records")
            self.json_publisher.publish(json)

        if self.get_parameter('pub_boxes').value:
            detections = self.getDetectionArray(results.pandas().xyxy[0])
            self.detection_publisher.publish(detections)


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
