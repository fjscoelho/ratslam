from rosbags.rosbag2 import Reader
import csv
import struct
from pathlib import Path

def extract_ViewTemplate(bag_path, topic, output_file):
    # Garante que o diretório de saída existe
    Path(output_file).parent.mkdir(parents=True, exist_ok=True)
    
    with Reader(bag_path) as reader, open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['stamp_sec', 'stamp_nsec','current_id', 'relative_rad', 'feature_count'])
        
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == topic:
                try:
                    # Offset 8: Pula o header (seq + stamp + frame_id)
                    # Assumindo estrutura:
                    # - current_id: uint32 (4 bytes)
                    # - relative_rad: double (8 bytes)
                    # - feature array length: uint32 (4 bytes)
                    
                    stamp_sec = struct.unpack('<I', rawdata[4:8])[0]  # <I = unsigned int little-endian
                    stamp_nsec = struct.unpack('<I', rawdata[8:12])[0]  # <I = unsigned int little-endian
                    current_id = struct.unpack('<I', rawdata[20:24])[0]  # <I = unsigned int little-endian
                    relative_rad = struct.unpack('<d', rawdata[24:32])[0]  # <d = double little-endian
                    feature_count = struct.unpack('<I', rawdata[32:36])[0]
                    
                    writer.writerow([stamp_sec, stamp_nsec, current_id, relative_rad, feature_count])
                    
                except Exception as e:
                    print(f"Erro processando mensagem em t={timestamp}: {str(e)[:200]}")

def extract_TopologicalAction(bag_path, topic, output_file):
    # Garante que o diretório de saída existe
    Path(output_file).parent.mkdir(parents=True, exist_ok=True)
    
    with Reader(bag_path) as reader, open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['stamp_sec', 'stamp_nsec', 'action', 'src_id', 'dest_id', 'relative_rad'])
        
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == topic:
                try:
                    # Offset 8: Pula o header (seq + stamp + frame_id)
                    # Assumindo estrutura:
                    # - current_id: uint32 (4 bytes)
                    # - relative_rad: double (8 bytes)
                    # - feature array length: uint32 (4 bytes)
                    
                    stamp_sec = struct.unpack('<I', rawdata[4:8])[0]  # <I = unsigned int little-endian
                    stamp_nsec = struct.unpack('<I', rawdata[8:12])[0]  # <I = unsigned int little-endian
                    action = struct.unpack('<I', rawdata[20:24])[0]  # <I = unsigned int little-endian
                    src_id = struct.unpack('<I', rawdata[24:28])[0]  # <I = unsigned int little-endian
                    dest_id = struct.unpack('<I', rawdata[28:32])[0]  # <I = unsigned int little-endian
                    relative_rad = struct.unpack('<d', rawdata[32:40])[0] # <d = double little-endian
                    
                    writer.writerow([stamp_sec, stamp_nsec, action, src_id, dest_id, relative_rad])
                    
                except Exception as e:
                    print(f"Erro processando mensagem em t={timestamp}: {str(e)[:200]}")

if __name__ == "__main__":

    try:
        print("⏳ Iniciando extração de dados ViewTemplate...")
        extract_ViewTemplate(
            'output_bags/surveyor_test1.bag',
            '/surveyor/LocalView/Template',
            'vt_id.csv'
        )
        print("✅ Extração concluída com sucesso!")
        print("📊 Estatísticas do arquivo gerado:")
        print(f"   - Tamanho: {Path('vt_id.csv').stat().st_size/1024:.2f} KB")
        print(f"   - Linhas: {sum(1 for _ in open('vt_id.csv'))}")

        print("⏳ Iniciando extração de dados TopologicalNode...")
        extract_TopologicalAction(
            'output_bags/surveyor_test1.bag',
            '/surveyor/PoseCell/TopologicalAction',
            'em_id.csv'
        )
        print("✅ Extração concluída com sucesso!")
        print("📊 Estatísticas do arquivo gerado:")
        print(f"   - Tamanho: {Path('em_id.csv').stat().st_size/1024:.2f} KB")
        print(f"   - Linhas: {sum(1 for _ in open('em_id.csv'))}")
    except Exception as e:
        print(f"❌ Erro fatal: {str(e)}")