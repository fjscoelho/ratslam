# Extracts GPS data from ROS 2 bag files and exports it to CSV format.

import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import NavSatFix
from pathlib import Path
import csv

def extract_gps_data(bag_path, topic_name, output_file):
    # Configurar leitor do bag
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    # Escrever CSV
    with open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['stamp_sec', 'stamp_nsec', 'latitude', 'longitude', 'altitude'])
        
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            if topic == topic_name:
                msg = deserialize_message(data, NavSatFix)
                writer.writerow([msg.header.stamp.sec, msg.header.stamp.nanosec, msg.latitude, msg.longitude, msg.altitude])
    
    print(f"Dados GPS extraídos para: {output_file}")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Extract GPS data from gps_fix message to CSV')
    parser.add_argument('bag_file', help='Path to folder that contains bag in .db3 format')
    parser.add_argument('--topic', default='/surveyor/gps_fix', help='Topic name to extract from bag')
    parser.add_argument('--gps_data', default='gps.csv', help='output file for gps data CSV')

    args = parser.parse_args()

    try:
        print("⏳ Starting gps_fix data extraction...")
        extract_gps_data(
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