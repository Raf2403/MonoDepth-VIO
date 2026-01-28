#!/usr/bin/env python3
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import sqlite3
import numpy as np
import sys

def extract_from_bag(bag_file, output_file, topic_name='/ov_msckf/poseimu'):
    """Extract trajectory directly from ROS2 bag"""
    
    # Connect to bag database
    conn = sqlite3.connect(bag_file if bag_file.endswith('.db3') else bag_file + '/db3')
    cursor = conn.cursor()
    
    # Find the topic
    cursor.execute("SELECT id, type FROM topics WHERE name=?", (topic_name,))
    topic_info = cursor.fetchone()
    
    if not topic_info:
        print(f"Topic {topic_name} not found in bag!")
        cursor.execute("SELECT name FROM topics")
        all_topics = cursor.fetchall()
        print("Available topics:")
        for t in all_topics:
            print(f"  {t[0]}")
        conn.close()
        return False
    
    topic_id, msg_type = topic_info
    msg_class = get_message(msg_type)
    
    # Get messages
    cursor.execute("""
        SELECT timestamp, data 
        FROM messages 
        WHERE topic_id = ? 
        ORDER BY timestamp
    """, (topic_id,))
    
    messages = cursor.fetchall()
    print(f"Found {len(messages)} messages in topic: {topic_name}")
    
    # Extract and save
    with open(output_file, 'w') as f:
        for timestamp, data in messages:
            try:
                msg = deserialize_message(data, msg_class)
                
                # Extract pose based on message type
                if hasattr(msg, 'pose') and hasattr(msg.pose, 'position'):
                    pose = msg.pose
                elif hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
                    pose = msg.pose.pose
                else:
                    continue
                
                # Convert timestamp (nanoseconds to seconds)
                if hasattr(msg.header.stamp, 'sec'):
                    ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                else:
                    ts = timestamp / 1e9
                
                f.write(f"{ts:.9f} ")
                f.write(f"{pose.position.x:.9f} {pose.position.y:.9f} {pose.position.z:.9f} ")
                f.write(f"{pose.orientation.x:.9f} {pose.orientation.y:.9f} ")
                f.write(f"{pose.orientation.z:.9f} {pose.orientation.w:.9f}\n")
                
            except Exception as e:
                print(f"Warning: Could not process message: {e}")
                continue
    
    conn.close()
    print(f"Trajectory saved to {output_file}")
    return True

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 extract_from_bag.py <bag_file.db3> <output_trajectory.txt>")
        print("\nExample: python3 extract_from_bag.py openvins_bag.db3 trajectory.txt")
        sys.exit(1)
    
    success = extract_from_bag(sys.argv[1], sys.argv[2])
    if not success:
        sys.exit(1)