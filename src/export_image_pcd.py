#!/usr/bin/env python
import rosbag
import rospy
import cv2
import numpy as np
import os
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import argparse
import pcl

def save_image(msg, directory, frame_num):
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
    # img_path = os.path.join(directory, f"frame{frame_num}.png")
    img_path = os.path.join(directory, f"{frame_num}.png")
    cv2.imwrite(img_path, img_rgb)

def save_pointcloud(msg, directory, frame_num):
    pc = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    pc_list = list(pc)
    
    cloud = pcl.PointCloud()
    cloud.from_list(pc_list)
    
    # pc_path = os.path.join(directory, f"frame{frame_num}.pcd")
    pc_path = os.path.join(directory, f"{frame_num}.pcd")
    pcl.save(cloud, pc_path)

def timestamps_close_enough(t1, t2, interval):
    return abs(t1.to_sec() - t2.to_sec()) < interval

def main(args):
    image_path = os.path.join(args.output_directory, "image")
    pcd_path = os.path.join(args.output_directory, "pcd")
    if (not os.path.exists(image_path)) and os.path.isdir(args.output_directory):
        os.makedirs(image_path)
    if (not os.path.exists(pcd_path)) and os.path.isdir(args.output_directory):
        os.makedirs(pcd_path)

    bag = rosbag.Bag(args.bag_file)
    frame_num = 0
    last_saved_time = None
    image_msg = None
    for topic, msg, t in bag.read_messages(topics=[args.image_topic, args.pointcloud_topic]):
        if last_saved_time is None:
            last_saved_time = t

        time_elapsed = t.to_sec() - last_saved_time.to_sec()
        
        if time_elapsed >= args.export_interval:
            
            if topic == args.image_topic:
                image_msg = msg
            elif topic == args.pointcloud_topic and image_msg is not None:
                if timestamps_close_enough(image_msg.header.stamp, msg.header.stamp, args.interval):
                    save_image(image_msg, image_path, frame_num)
                    save_pointcloud(msg, pcd_path, frame_num)
                    print("time stamp: {},{} ".format(image_msg.header.stamp.to_sec(), msg.header.stamp.to_sec()))
                    frame_num += 1
                    image_msg = None  # reset the image message
                    last_saved_time = t

    bag.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Export images and pointclouds from rosbag.")
    parser.add_argument("bag_file", type=str, help="Path to the rosbag file.")
    parser.add_argument("image_topic", type=str, help="Image topic name.")
    parser.add_argument("pointcloud_topic", type=str, help="PointCloud topic name.")
    parser.add_argument("output_directory", type=str, help="Directory to save the images and pointclouds.")
    parser.add_argument("--interval", type=float, default=0.01, help="Max time difference between image and point cloud messages (in seconds).")
    parser.add_argument("--export_interval", type=float, default=2.0, help="Interval in seconds to save the frames.")
    args = parser.parse_args()
    main(args)