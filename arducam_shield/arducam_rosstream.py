import cv2
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from arducam_shield.arducam import ArducamCamera, ImageConverter


class ImageStream(Node):

    def __init__(self):
        super().__init__("image_stream")
        # ros parameters
        self.declare_parameter("cfg_file")
        self.declare_parameter("calib_file")
        self.declare_parameter("cam_index", 0)
        self.declare_parameter("rotate_180", False)

        # message
        self.frame = self.get_namespace() + "/camera_color_optical_frame"
        self.infomsg = self.compose_camera_info(self.get_parameter("calib_file").value)
        self.imgpub = self.create_publisher(Image, "image_raw", 1)
        self.infopub = self.create_publisher(CameraInfo, "camera_info", 1)
        self.timer = self.create_timer(0.01, callback=self.timer_callback)

        # hardware interface
        self.br = CvBridge()
        self.camera = ArducamCamera()
        self.exit_ = False
        if not self.camera.openCamera(fname=self.get_parameter("cfg_file").value, index=self.get_parameter("cam_index").value):
            raise RuntimeError("Failed to open camera.")
        self.camera.start()

    def timer_callback(self):
        timenow = self.get_clock().now().to_msg()
        if not self.exit_:
            ret, data, cfg = self.camera.read()
            if ret:
                image = ImageConverter.convert_image(data, cfg, self.camera.color_mode)

                # rotate image if it is rotated
                if self.get_parameter("rotate_180").value:
                    image = cv2.rotate(image, cv2.ROTATE_180)

                imgmsg = self.br.cv2_to_imgmsg(image, encoding="bgr8")
                imgmsg.header.stamp = timenow
                imgmsg.header.frame_id = self.frame
                self.imgpub.publish(imgmsg)
                self.infomsg.header.stamp = timenow
                self.infopub.publish(self.infomsg)

    def compose_camera_info(self, yamlPath):
        infomsg = CameraInfo()
        try:
            with open(yamlPath, "r") as f:
                p = yaml.load(f, yaml.FullLoader)
            infomsg.header.frame_id = self.frame
            infomsg.height = p["image_height"]
            infomsg.width = p["image_width"]
            infomsg.distortion_model = p["distortion_model"]
            infomsg.d = p["distortion_coefficients"]["data"]
            infomsg.k = p["camera_matrix"]["data"]
            infomsg.r = p["rectification_matrix"]["data"]
            infomsg.p = p["projection_matrix"]["data"]
        except:
            self.get_logger().info("No calibration file is provide. Assume camera is uncalibrated")
        return infomsg

    def close_cam(self):
        self.camera.stop()
        self.camera.closeCamera()
        self.get_logger().info("Camera Closed")


def main(args=None):
    rclpy.init(args=args)
    imageNode = ImageStream()
    try:
        rclpy.spin(imageNode)
    except KeyboardInterrupt:
        imageNode.exit_ = True
        imageNode.close_cam()
    finally:
        imageNode.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
