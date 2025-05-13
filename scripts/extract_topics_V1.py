from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
import csv
import numpy as np

# 1. Configuração do Typestore
typestore = get_typestore(Stores.ROS2_HUMBLE)

# 2. Definição dos tipos no formato CORRETO
types_def = {
    'std_msgs/msg/Header': (
        [],
        [
            ('seq', 'uint32'),
            ('stamp', ('int32', 'uint32')),  # sec, nanosec
            ('frame_id', 'string')
        ]
    ),
    'topological_msgs/msg/ViewTemplate': (
        [],
        [
            ('header', 'std_msgs/msg/Header'),
            ('current_id', 'uint32'),
            ('relative_rad', 'float64'),
            ('feature', 'float64[]'),
            ('elapsed_time', 'float64[]')
        ]
    )
}

# 3. Registro dos tipos
typestore.register(types_def)

def export_viewtemplate(bag_path, topic, output_file):
    with Reader(bag_path) as reader, open(output_file, 'w', newline='') as csvfile:
        writer = None
        
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == topic:
                try:
                    # 4. Desserialização
                    msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                    
                    # 5. Cabeçalho do CSV
                    if writer is None:
                        fieldnames = [
                            'ros_timestamp',
                            'header_seq',
                            'header_stamp_sec',
                            'header_stamp_nanosec',
                            'header_frame_id',
                            'current_id',
                            'relative_rad',
                            'feature_count',
                            'elapsed_time_count',
                            'feature_first',
                            'elapsed_first'
                        ]
                        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                        writer.writeheader()
                    
                    # 6. Processamento seguro dos arrays
                    feature_first = msg.feature[0] if len(msg.feature) > 0 else 0.0
                    elapsed_first = msg.elapsed_time[0] if len(msg.elapsed_time) > 0 else 0.0
                    
                    # 7. Escrita no CSV
                    writer.writerow({
                        'ros_timestamp': timestamp,
                        'header_seq': msg.header.seq,
                        'header_stamp_sec': msg.header.stamp[0],
                        'header_stamp_nanosec': msg.header.stamp[1],
                        'header_frame_id': msg.header.frame_id,
                        'current_id': msg.current_id,
                        'relative_rad': msg.relative_rad,
                        'feature_count': len(msg.feature),
                        'elapsed_time_count': len(msg.elapsed_time),
                        'feature_first': feature_first,
                        'elapsed_first': elapsed_first
                    })
                    
                except Exception as e:
                    print(f"Erro processando mensagem: {str(e)[:200]}")

if __name__ == "__main__":
    try:
        print("⏳ Iniciando extração de dados...")
        export_viewtemplate(
            'output_bags/surveyor_test1.bag',
            '/surveyor/LocalView/Template',
            'vt_data_final.csv'
        )
        print("✅ Extração concluída com sucesso! Arquivo 'vt_data_final.csv' gerado.")
    except Exception as e:
        print(f"❌ Falha na extração: {str(e)}")