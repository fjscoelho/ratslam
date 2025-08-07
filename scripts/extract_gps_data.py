# Extracts GPS data from ROS 2 bag files and exports it to CSV format.

from rosbags.rosbag2 import Reader
import csv
import struct
from pathlib import Path
from sensor_msgs.msg import NavSatFix
from rclpy.serialization import deserialize_message
import argparse

# Example args:
# python3 extract_gps_data.py ../../../bags/20250416_131706_bag_ros2 --topic /surveyor/gps_fix --gps_data exported_data/gps.csv

def extract_gps(bag_path, topic, output_file):
    Path(output_file).parent.mkdir(parents=True, exist_ok=True)
    
    with Reader(bag_path) as reader, open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['stamp_sec', 'stamp_nsec', 'status','service','latitude', 'longitude'])
        
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == topic:
                try:
                    
                    deserialized_msg = deserialize_message(rawdata, NavSatFix)

                    writer.writerow([deserialized_msg.header.stamp.sec, deserialized_msg.header.stamp.nanosec,
                                     deserialized_msg.status.status, deserialized_msg.status.service,
                                     deserialized_msg.latitude, deserialized_msg.longitude])

                except Exception as e:
                    print(f"Erro processando mensagem RobotPose em t={timestamp}: {str(e)[:200]}")

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Extract GPS data from gps_fix message to CSV')
    parser.add_argument('bag_file', help='Path to folder that contains bag in .db3 format')
    parser.add_argument('--topic', default='/surveyor/gps_fix', help='Topic name to extract from bag')
    parser.add_argument('--gps_data', default='gps.csv', help='output file for gps data CSV')

    args = parser.parse_args()

    try:
        print("⏳ Starting gps_fix data extraction...")
        extract_gps(
            args.bag_file,
            args.topic,
            args.gps_data
        )
        print("✅ Extraction completed successfully!")
        print("📊 Generated file statistics:")
        print(f"   - Length: {Path(args.gps_data).stat().st_size/1024:.2f} KB")
        print(f"   - Lines: {sum(1 for _ in open(args.gps_data))}")

    except Exception as e:
        print(f"❌ Erro fatal: {str(e)}")