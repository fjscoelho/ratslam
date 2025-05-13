from rosbags.rosbag2 import Reader
import csv
import struct
from pathlib import Path

def extract_ViewTemplate(bag_path, topic, output_file):
    # Ensure that the output directory exists 
    Path(output_file).parent.mkdir(parents=True, exist_ok=True)
    
    with Reader(bag_path) as reader, open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['stamp_sec', 'stamp_nsec','current_id', 'relative_rad', 'feature_count'])
        
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == topic:
                try:
                    # Data structure
                    # header:
                    #   stamp:
                    #       sec: uint32 (4 bytes)
                    #       nanosec: uint32 (4 bytes)
                    #   frame_id: (empty) (8 bytes)
                    # current_id: uint32 (4 bytes)
                    # relative_rad: float64 (8 bytes)
                    # feature: float64[]
                    # elapsed_time: float64[]
                    
                    stamp_sec = struct.unpack('<I', rawdata[4:8])[0]  # <I = unsigned int little-endian
                    stamp_nsec = struct.unpack('<I', rawdata[8:12])[0]  # <I = unsigned int little-endian
                    current_id = struct.unpack('<I', rawdata[20:24])[0]  # <I = unsigned int little-endian
                    relative_rad = struct.unpack('<d', rawdata[24:32])[0]  # <d = double little-endian
                    feature_count = struct.unpack('<I', rawdata[32:36])[0]
                    
                    writer.writerow([stamp_sec, stamp_nsec, current_id, relative_rad, feature_count])
                    
                except Exception as e:
                    print(f"Erro processando mensagem ViewTemplate em t={timestamp}: {str(e)[:200]}")

def extract_TopologicalAction(bag_path, topic, output_file):
    Path(output_file).parent.mkdir(parents=True, exist_ok=True)
    
    with Reader(bag_path) as reader, open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['stamp_sec', 'stamp_nsec', 'action', 'src_id', 'dest_id', 'relative_rad'])
        
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == topic:
                try:
                    # Data structure
                    # header:
                    #   stamp:
                    #       sec: uint32 (4 bytes)
                    #       nanosec: uint32 (4 bytes)
                    #   frame_id: (empty) (8 bytes)
                    # action: uint32 (4 bytes)
                    # src_id: uint32 (4 bytes)
                    # dest_id: uint32 (4 bytes)
                    # relative_rad: float64 (8 bytes)
                   
                    stamp_sec = struct.unpack('<I', rawdata[4:8])[0]  # <I = unsigned int little-endian
                    stamp_nsec = struct.unpack('<I', rawdata[8:12])[0]  # <I = unsigned int little-endian
                    action = struct.unpack('<I', rawdata[20:24])[0]  # <I = unsigned int little-endian
                    src_id = struct.unpack('<I', rawdata[24:28])[0]  # <I = unsigned int little-endian
                    dest_id = struct.unpack('<I', rawdata[28:32])[0]  # <I = unsigned int little-endian
                    relative_rad = struct.unpack('<d', rawdata[32:40])[0] # <d = double little-endian
                    
                    writer.writerow([stamp_sec, stamp_nsec, action, src_id, dest_id, relative_rad])
                    
                except Exception as e:
                    print(f"Erro processando mensagem TopologicalAction em t={timestamp}: {str(e)[:200]}")

def extract_RobotPose(bag_path, topic, output_file):
    Path(output_file).parent.mkdir(parents=True, exist_ok=True)
    
    with Reader(bag_path) as reader, open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['stamp_sec', 'stamp_nsec', 'pos_x', 'pos_y', 'pos_z', 'ori_x', 'ori_y', 'ori_z', 'ori_w'])
        
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == topic:
                try:
                    # Data structure
                    # header:
                    #   stamp:
                    #       sec: uint32 (4 bytes)
                    #       nanosec: uint32 (4 bytes)
                    #   frame_id: (empty) (8 bytes)
                    # pose
                    #   position
                    #       x: float64 (8 bytes)
                    #       y: float64 (8 bytes)
                    #       z: float64 (8 bytes)
                    #   orientation
                    #       x: float64 (8 bytes)
                    #       y: float64 (8 bytes)
                    #       z: float64 (8 bytes)
                    #       w: float64 (8 bytes)
                   
                    stamp_sec = struct.unpack('<I', rawdata[4:8])[0]  # <I = unsigned int little-endian
                    stamp_nsec = struct.unpack('<I', rawdata[8:12])[0]  # <I = unsigned int little-endian
                    pos_x = struct.unpack('<d', rawdata[20:28])[0]  # <d = double little-endian
                    pos_y = struct.unpack('<d', rawdata[28:36])[0]  # <d = double little-endian
                    pos_z = struct.unpack('<d', rawdata[36:44])[0]  # <d = double little-endian
                    ori_x = struct.unpack('<d', rawdata[44:52])[0]  # <d = double little-endian
                    ori_y = struct.unpack('<d', rawdata[52:60])[0]  # <d = double little-endian
                    ori_z = struct.unpack('<d', rawdata[60:68])[0]  # <d = double little-endian
                    ori_w = struct.unpack('<d', rawdata[68:76])[0]  # <d = double little-endian
                    writer.writerow([stamp_sec, stamp_nsec, pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w])
                    
                except Exception as e:
                    print(f"Erro processando mensagem RobotPose em t={timestamp}: {str(e)[:200]}")

if __name__ == "__main__":

    try:
        print("⏳ Iniciando extração de dados ViewTemplate...")
        extract_ViewTemplate(
            'output_bags/surveyor_test1.bag',
            '/surveyor/LocalView/Template',
            'exported_data/vt_id.csv'
        )
        print("✅ Extração concluída com sucesso!")
        print("📊 Estatísticas do arquivo gerado:")
        print(f"   - Tamanho: {Path('exported_data/vt_id.csv').stat().st_size/1024:.2f} KB")
        print(f"   - Linhas: {sum(1 for _ in open('exported_data/vt_id.csv'))}")

        print("⏳ Iniciando extração de dados TopologicalNode...")
        extract_TopologicalAction(
            'output_bags/surveyor_test1.bag',
            '/surveyor/PoseCell/TopologicalAction',
            'exported_data/em_id.csv'
        )
        print("✅ Extração concluída com sucesso!")
        print("📊 Estatísticas do arquivo gerado:")
        print(f"   - Tamanho: {Path('exported_data/em_id.csv').stat().st_size/1024:.2f} KB")
        print(f"   - Linhas: {sum(1 for _ in open('exported_data/em_id.csv'))}")

        print("⏳ Iniciando extração de dados RobotPose...")
        extract_RobotPose(
            'output_bags/surveyor_test1.bag',
            '/surveyor/ExperienceMap/RobotPose',
            'exported_data/pose.csv'
        )
        print("✅ Extração concluída com sucesso!")
        print("📊 Estatísticas do arquivo gerado:")
        print(f"   - Tamanho: {Path('exported_data/pose.csv').stat().st_size/1024:.2f} KB")
        print(f"   - Linhas: {sum(1 for _ in open('exported_data/pose.csv'))}")
    except Exception as e:
        print(f"❌ Erro fatal: {str(e)}")