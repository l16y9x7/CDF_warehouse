"""ROS camera names shared with Tianji; HTTP keeps the business role names."""

ROS_CAMERA_IDS = {"head": "head", "hand_left": "left_wrist", "hand_right": "right_wrist"}


def ros_camera_id(camera):
    return ROS_CAMERA_IDS[camera]
