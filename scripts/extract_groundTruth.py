from rosbags.rosbag2 import Reader
import csv
import struct
from pathlib import Path
from sensor_msgs.msg import NavSatFix
from rclpy.serialization import deserialize_message

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

    try:
        print("⏳ Iniciando extração de dados gps_fix...")
        extract_gps(
            '../../../bags/20250416_131706_bag_ros2',
            '/surveyor/gps_fix',
            'exported_data/ground_truth.csv'
        )
        print("✅ Extração concluída com sucesso!")
        print("📊 Estatísticas do arquivo gerado:")
        print(f"   - Tamanho: {Path('exported_data/ground_truth.csv').stat().st_size/1024:.2f} KB")
        print(f"   - Linhas: {sum(1 for _ in open('exported_data/ground_truth.csv'))}")

    except Exception as e:
        print(f"❌ Erro fatal: {str(e)}")