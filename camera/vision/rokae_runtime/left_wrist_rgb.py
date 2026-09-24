"""Publish only D405 RGB at 1280x720@15; systemd owns the USB device."""
from array import array
import argparse
import json
from pathlib import Path


def configure_pipeline(rs, serial):
    config = rs.config()
    config.disable_all_streams()
    config.enable_device(serial)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15)
    return config


def main():
    import pyrealsense2 as rs
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    args = parser.parse_args()
    settings = json.loads(Path(args.config).read_text())
    camera = settings['rokae']['owner']['cameras']['hand_left']
    if not camera.get('enabled') or not camera.get('color_only'):
        raise ValueError('Left wrist must be enabled in RGB-only mode')
    serial = camera['match']['value']
    if not serial:
        raise ValueError('D405 serial number is required')
    rclpy.init()
    node = Node('vision_left_wrist_rgb')
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
    publisher = node.create_publisher(Image, '/camera/left_wrist/color/image_raw', qos)
    pipeline = rs.pipeline()
    started = False
    try:
        profile = pipeline.start(configure_pipeline(rs, serial))
        started = True
        streams = profile.get_streams()
        if len(streams) != 1 or streams[0].stream_type() != rs.stream.color:
            raise RuntimeError('Unexpected non-color stream; stopping camera')
        video = streams[0].as_video_stream_profile()
        if (video.width(), video.height(), video.fps()) != (1280, 720, 15):
            raise RuntimeError('Unexpected RGB profile; stopping camera')
        node.get_logger().info(f'D405 {serial}: COLOR ONLY bgr8 1280x720@15; depth disabled')
        while rclpy.ok():
            color = pipeline.wait_for_frames(3000).get_color_frame()
            if not color:
                continue
            message = Image()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = 'left_wrist_color_optical_frame'
            message.height, message.width = color.get_height(), color.get_width()
            message.encoding = 'bgr8'
            message.is_bigendian = 0
            message.step = color.get_stride_in_bytes()
            message.data = array('B', bytes(color.get_data()))
            publisher.publish(message)
    except KeyboardInterrupt:
        pass
    finally:
        if started:
            pipeline.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
