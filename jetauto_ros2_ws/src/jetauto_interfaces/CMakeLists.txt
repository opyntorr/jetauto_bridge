cmake_minimum_required(VERSION 3.8)
project(jetauto_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ColorDetect.msg"
  "msg/ColorInfo.msg"
  "msg/ColorsInfo.msg"
  "msg/Float64List.msg"
  "msg/LineROI.msg"
  "msg/ObjectInfo.msg"
  "msg/ObjectsInfo.msg"
  "msg/PixelPosition.msg"
  "msg/Point2D.msg"
  "msg/Points.msg"
  "msg/Pose2D.msg"
  "msg/ROI.msg"
  "srv/GetPose.srv"
  "srv/SetCircleROI.srv"
  "srv/SetColorDetectParam.srv"
  "srv/SetColorRGBA.srv"
  "srv/SetFloat64.srv"
  "srv/SetFloat64List.srv"
  "srv/SetInt64.srv"
  "srv/SetJointValue.srv"
  "srv/SetLineROI.srv"
  "srv/SetPoint.srv"
  "srv/SetPose.srv"
  "srv/SetPose2D.srv"
  "srv/SetString.srv"
  "srv/SetStringList.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
