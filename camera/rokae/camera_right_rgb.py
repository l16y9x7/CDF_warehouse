#!/usr/bin/env python3
"""Right wrist RGB-only source. Uses the existing camera config and ROS domain."""
from array import array
import argparse
import signal
import threading
import logging
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from vision.rokae_runtime.realsense_color_publisher import load_source
from vision.rokae_runtime.driver_supervisor import validate_realsense_color_profile


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--config',required=True)
    args=parser.parse_args()
    _, cfg=load_source(args.config,'hand_right')
    width,height,fps=validate_realsense_color_profile(cfg,rs)
    serial=cfg['match']['value']
    stop=threading.Event()
    for signum in (signal.SIGINT,signal.SIGTERM):
        signal.signal(signum,lambda *_:stop.set())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=Node('mui_right_wrist_rgb')
    publisher=node.create_publisher(Image,'/camera/right_wrist/color/image_raw',qos_profile_sensor_data)
    pipeline=rs.pipeline(); settings=rs.config()
    settings.disable_all_streams();settings.enable_device(serial)
    settings.enable_stream(rs.stream.color,width,height,rs.format.rgb8,fps)
    started=False
    try:
        profile=pipeline.start(settings);started=True
        streams=profile.get_streams()
        if len(streams)!=1 or streams[0].stream_type()!=rs.stream.color:
            raise RuntimeError('Unexpected stream; right wrist must be RGB only')
        node.get_logger().info(f'Right wrist {serial}: RGB8 {width}x{height}@{fps}; depth disabled')
        while not stop.is_set() and rclpy.ok():
            color=pipeline.wait_for_frames(3000).get_color_frame()
            if not color:continue
            if (color.get_width(),color.get_height())!=(width,height):
                raise RuntimeError('Unexpected RGB resolution')
            message=Image()
            message.header.stamp=node.get_clock().now().to_msg()
            message.header.frame_id='right_wrist_color_optical_frame'
            message.height,message.width=height,width
            message.encoding='rgb8';message.is_bigendian=0
            message.step=color.get_stride_in_bytes()
            # array avoids per-byte Python validation on multi-megabyte RGB frames.
            message.data=array('B',bytes(color.get_data()))
            publisher.publish(message)
    finally:
        if started:pipeline.stop()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
