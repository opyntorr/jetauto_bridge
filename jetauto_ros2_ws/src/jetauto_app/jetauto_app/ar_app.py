#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/03/24
# @author:aiden
"""
Augmented-reality (AR) demo. ROS2 (rclpy) port of jetauto_app/scripts/ar_app.py.

Detects AprilTags (tag36h11) in the camera image, solves the tag pose with
solvePnP, and overlays either a wireframe cube ('rectangle') or a loaded .obj
model (bicycle/fox/chair/cow/wolf) onto the tag.

This is a vision-overlay app for the BASE JetAuto: no arm, no bus servos, and
no chassis motion (it publishes no cmd_vel).

Services (under /ar_app):
  ~/enter (Trigger)         subscribe to the camera image + camera_info, start
  ~/exit  (Trigger)         stop and unsubscribe
  ~/set_model (SetString)   data='' clears; otherwise 'rectangle' or a model name
  ~/heartbeat (SetBool)     watchdog -> _stop()
Publishes: ~/image_result (sensor_msgs/Image, rgb8)
Params: image_topic (default /camera/rgb/image_raw), model_path,
        machine_type (default JetAuto), lidar_type (default A1),
        cmd_vel_topic (default cmd_vel).
"""
import os
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import apriltag
from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from jetauto_interfaces.srv import SetString

from jetauto_app.common import Heart
from jetauto_app.objloader_simple import OBJ as obj_load


def _default_model_path():
    # Default to the installed share/jetauto_app/models; fall back to the source tree.
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('jetauto_app'), 'models')
    except Exception:
        return os.path.join(
            os.path.abspath(os.path.join(os.path.split(os.path.realpath(__file__))[0], '..')),
            'models')


# 求解pnp的点，正方形的四个角和中心点(solve for the points of pnp, the four corners and the center point of the square)
OBJP = np.array([[-1, -1,  0],
                 [ 1, -1,  0],
                 [-1,  1,  0],
                 [ 1,  1,  0],
                 [ 0,  0,  0]], dtype=np.float32)

# 绘制立方体的坐标(draw the coordinate of the cube)
AXIS = np.float32([[-1, -1, 0],
                   [-1,  1, 0],
                   [ 1,  1, 0],
                   [ 1, -1, 0],
                   [-1, -1, 2],
                   [-1,  1, 2],
                   [ 1,  1, 2],
                   [ 1, -1, 2]])

# 模型缩放比例(model scaling)
MODELS_SCALE = {
                'bicycle': 50,
                'fox': 4,
                'chair': 400,
                'cow': 0.4,
                'wolf': 0.6,
                }


def draw_rectangle(img, imgpts):
    '''
    绘制立方体(draw the cube)
    :param img: 要绘制立方体的图像(the image to draw the cube)
    :param imgpts: 立方体的角点(angular point of the cube)
    :return: 要绘制立方体的图像(the image to draw the cube)
    '''
    imgpts = np.int32(imgpts).reshape(-1, 2)
    cv2.drawContours(img, [imgpts[:4]], -1, (0, 255, 0), -3)  # 绘制轮廓点，填充形式
    for i, j in zip(range(4), range(4, 8)):
        cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]), (255), 3)  # 绘制线连接点
    cv2.drawContours(img, [imgpts[4:]], -1, (0, 0, 255), 3)  # 绘制轮廓点，不填充

    return img


class ARNode(Node):
    def __init__(self):
        super().__init__('ar_app')
        # 声明参数(declare parameters)
        self.declare_parameter('image_topic', '/camera/rgb/image_raw')
        self.declare_parameter('model_path', _default_model_path())
        self.declare_parameter('machine_type', 'JetAuto')
        self.declare_parameter('lidar_type', 'A1')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.image_topic = self.get_parameter('image_topic').value
        self.model_path = self.get_parameter('model_path').value
        self.machine_type = self.get_parameter('machine_type').value
        self.lidar_type = self.get_parameter('lidar_type').value

        # 摄像头内参(default camera intrinsics; overwritten by camera_info)
        self.camera_intrinsic = np.matrix([[619.063979, 0,          302.560920],
                                           [0,          613.745352, 237.714934],
                                           [0,          0,          1]])
        self.dist_coeffs = np.array([0.103085, -0.175586, -0.001190, -0.007046, 0.000000])

        self.obj = None
        self.image_sub = None
        self.target_model = None
        self.camera_info_sub = None

        self.bridge = CvBridge()
        # apriltag 0.0.16 (Swatbotics) API: Detector(DetectorOptions(...))
        self.tag_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11'))
        self.lock = threading.RLock()  # 线程锁(thread lock)

        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)  # 发布最终图像(publish the final image)
        self.create_service(Trigger, '~/enter', self.enter_srv_callback)  # 进入玩法服务(enter the game service)
        self.create_service(Trigger, '~/exit', self.exit_srv_callback)  # 退出玩法服务(exit the game service)
        self.create_service(SetString, '~/set_model', self.set_model_srv_callback)  # 设置模型服务(set the model service)
        self.heart = Heart(self, '~/heartbeat', 5, lambda _: self._stop())  # 心跳包(heartbeat package)
        self.get_logger().info('ar_app listo (machine_type=%s, lidar_type=%s)' % (self.machine_type, self.lidar_type))

    def _camera_info_topic(self):
        # Derive the camera_info topic from the image topic (.../image_raw -> .../camera_info)
        base = self.image_topic.rsplit('/', 1)[0]
        return base + '/camera_info'

    def _unsubscribe(self):
        if self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
        if self.camera_info_sub is not None:
            self.destroy_subscription(self.camera_info_sub)
            self.camera_info_sub = None

    def _stop(self):
        # 复位状态并注销订阅(reset state and unsubscribe; mirrors the original heartbeat->exit)
        with self.lock:
            self.obj = None
            self.target_model = None
            self._unsubscribe()

    def enter_srv_callback(self, request, response):
        # 进入服务(enter the service)
        self.get_logger().info("ar enter")
        with self.lock:
            self._unsubscribe()
            self.obj = None
            self.target_model = None
            self.image_sub = self.create_subscription(
                Image, self.image_topic, self.image_callback, qos_profile_sensor_data)  # 订阅图像(subscribe to the image)
            self.camera_info_sub = self.create_subscription(
                CameraInfo, self._camera_info_topic(), self.camera_info_callback, qos_profile_sensor_data)  # 订阅摄像头信息(subscribe to the camera information)
        response.success = True
        return response

    def exit_srv_callback(self, request, response):
        # 退出服务(exit the service)
        self.get_logger().info("ar exit")
        self._stop()
        response.success = True
        return response

    def set_model_srv_callback(self, request, response):
        # 设置模型(set the model)
        with self.lock:
            self.get_logger().info("set model {}".format(request.data))
            if request.data == "":
                self.target_model = None
            else:
                self.target_model = request.data
                if self.target_model != 'rectangle':  # 如果不是绘制立方体(if the cube is not being drawn)
                    # 加载模型(load the model)
                    obj = obj_load(os.path.join(self.model_path, self.target_model + '.obj'), swapyz=True)
                    obj.faces = obj.faces[::-1]
                    new_faces = []
                    # 对模型进行解析，获取点坐标(analyze the model and get the point coordinates)
                    for face in obj.faces:
                        face_vertices = face[0]
                        points = []
                        colors = []
                        for vertex in face_vertices:
                            data = obj.vertices[vertex - 1]
                            points.append(data[:3])
                            if self.target_model != 'cow' and self.target_model != 'wolf':
                                colors.append(data[3:])
                        scale_matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) * MODELS_SCALE[self.target_model]  # 缩放(scale)
                        points = np.dot(np.array(points), scale_matrix)
                        if self.target_model == 'bicycle':
                            points = np.array([[p[0] - 670, p[1] - 350, p[2]] for p in points])
                            points = R.from_euler('xyz', (0, 0, 180), degrees=True).apply(points)
                        elif self.target_model == 'fox':
                            points = np.array([[p[0], p[1], p[2]] for p in points])
                            points = R.from_euler('xyz', (0, 0, -90), degrees=True).apply(points)
                        elif self.target_model == 'chair':
                            points = np.array([[p[0], p[1], p[2]] for p in points])
                            points = R.from_euler('xyz', (0, 0, -90), degrees=True).apply(points)
                        else:
                            points = np.array([[p[0], p[1], p[2]] for p in points])
                        if len(colors) > 0:
                            color = tuple(255 * np.array(colors[0]))
                        else:
                            color = None
                        new_faces.append((points, color))
                    self.obj = new_faces
        response.success = True
        return response

    def camera_info_callback(self, msg):
        # 摄像头内参信息获取回调(camera internal parameter callback)
        with self.lock:
            self.camera_intrinsic = np.array(msg.k).reshape(3, -1)
            self.dist_coeffs = np.array(msg.d)

    def image_callback(self, ros_image):
        # 图像回调(image callback)
        # 将ROS图像消息转化为numpy格式(convert ROS image into numpy format)
        rgb_image = self.bridge.imgmsg_to_cv2(ros_image, 'rgb8')
        result_image = np.copy(rgb_image)
        with self.lock:
            try:
                # 图像处理(process image)
                result_image = self.image_proc(rgb_image, result_image)
            except Exception as e:
                self.get_logger().error(str(e))
        # opencv格式转为ros格式(convert opencv format into ros format)
        ros_result = self.bridge.cv2_to_imgmsg(np.array(result_image, dtype=np.uint8), 'rgb8')
        ros_result.header = ros_image.header
        self.result_publisher.publish(ros_result)  # 发布图像(publish image)

    def image_proc(self, rgb_image, result_image):
        # 图像处理(process image)
        if self.target_model is not None:
            gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)  # 转为灰度图(convert into gray image)
            detections = self.tag_detector.detect(gray)  # aprilatg识别(aprilatg recognition)
            if len(detections) > 0:
                for detection in detections:  # 遍历(traverse)
                    # apriltag 0.0.16 Detection: .center (2,) and .corners (4x2, order lb,rb,rt,lt)
                    tag_center = detection.center
                    tag_corners = detection.corners
                    lb = tag_corners[0]
                    rb = tag_corners[1]
                    rt = tag_corners[2]
                    lt = tag_corners[3]
                    # 绘制四个角点(draw four angular points)
                    cv2.circle(result_image, (int(lb[0]), int(lb[1])), 2, (0, 255, 255), -1)
                    cv2.circle(result_image, (int(lt[0]), int(lt[1])), 2, (0, 255, 255), -1)
                    cv2.circle(result_image, (int(rb[0]), int(rb[1])), 2, (0, 255, 255), -1)
                    cv2.circle(result_image, (int(rt[0]), int(rt[1])), 2, (0, 255, 255), -1)
                    # cv2.circle(result_image, (int(tag_center[0]), int(tag_center[1])), 3, (255, 0, 0), -1)
                    corners = np.array([lb, rb, lt, rt, tag_center]).reshape(5, -1)
                    # 使用世界坐标系k个点坐标(OBJP)，对应图像坐标系2D的k个点坐标(corners)，以及相机内参camera_intrinsic和dist_coeffs进行反推图片的外参r, t
                    # (Use the world coordinate system k point coordinates (OBJP), the k point coordinates (corners) corresponding to the 2D image coordinate system,
                    # and the camera internal parameters camera_intrinsic and dist_coeffs to reverse the external parameters r, t of the picture)
                    ret, rvecs, tvecs = cv2.solvePnP(OBJP, corners, self.camera_intrinsic, self.dist_coeffs)
                    if self.target_model == 'rectangle':  # 如果要显示立方体则单独处理(if the cube needs to be displayed, process independently)
                        # 反向投影将世界坐标系点转换到图像点(backprojection converts world coordinate system points to image points)
                        imgpts, jac = cv2.projectPoints(AXIS, rvecs, tvecs, self.camera_intrinsic, self.dist_coeffs)
                        result_image = draw_rectangle(result_image, imgpts)
                    else:
                        for points, color in self.obj:
                            dst, jac = cv2.projectPoints(points.reshape(-1, 1, 3) / 100.0, rvecs, tvecs, self.camera_intrinsic, self.dist_coeffs)
                            imgpts = dst.astype(int)
                            # 手动上色(manual coloring)
                            if self.target_model == 'cow':
                                cv2.fillConvexPoly(result_image, imgpts, (0, 255, 255))
                            elif self.target_model == 'wolf':
                                cv2.fillConvexPoly(result_image, imgpts, (255, 255, 0))
                            else:
                                cv2.fillConvexPoly(result_image, imgpts, color)

        return result_image


def main(args=None):
    rclpy.init(args=args)
    node = ARNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
