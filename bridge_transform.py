import time
import tf2_ros, rospy
import tf2_geometry_msgs

from vision_msgs.msg import Detection2DArray
from agimus_controller_ros.controller_base import find_tracked_object


class VisionListener:
    def __init__(self, topic_name):
        # rospy.init_node("vision_listener_hpp", anonymous=True)
        self.topic_name = topic_name
        time.sleep(0.5)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.in_world_pose_object = None

        self.vision_subscriber = rospy.Subscriber(
            topic_name, Detection2DArray, self.vision_callback
        )

    def vision_callback(self, vision_msg: Detection2DArray):
        if vision_msg.detections == []:
            return
        in_camera_pose_object = find_tracked_object(vision_msg.detections)
        if in_camera_pose_object is None:
            return
        image_timestamp = vision_msg.detections[0].header.stamp
        in_world_M_camera = self.tf_buffer.lookup_transform(
            target_frame="world",
            source_frame="camera_color_optical_frame",
            time=image_timestamp,
        )
        self.in_world_pose_object = tf2_geometry_msgs.do_transform_pose(
            in_camera_pose_object, in_world_M_camera
        ).pose
