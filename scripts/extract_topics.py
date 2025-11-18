#!/usr/bin/env python3

import csv
import struct
from pathlib import Path
import argparse
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
# from rosbag2_py import Reader

# Importar os tipos de mensagem que você está usando
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from topological_msgs.msg import ViewTemplate, TopologicalAction

def extract_ViewTemplate(bag_path, topic, output_file):
    # Ensure that the output directory exists 
    Path(output_file).parent.mkdir(parents=True, exist_ok=True)
    
    # Configurar leitor do bag
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    with open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['stamp_sec', 'stamp_nsec','current_id', 'relative_rad', 'feature_count'])
        
        while reader.has_next():
            topic_name, data, timestamp_ns = reader.read_next()
            if topic_name == topic:
                try:
                    # Converter para segundos e nanosegundos
                    # stamp_sec = timestamp_ns // 1_000_000_000
                    # stamp_nsec = timestamp_ns % 1_000_000_000
                    
                    msg = deserialize_message(data, ViewTemplate)
                    stamp_sec = msg.header.stamp.sec
                    stamp_nsec = msg.header.stamp.nanosec
                    current_id = msg.current_id
                    relative_rad = msg.relative_rad
                    feature_count = len(msg.feature) if hasattr(msg, 'feature') else 0
                    
                    
                    writer.writerow([stamp_sec, stamp_nsec, current_id, relative_rad, feature_count])
                    
                except Exception as e:
                    print(f"Erro processando mensagem ViewTemplate: {str(e)[:200]}")

def extract_TopologicalAction(bag_path, topic, output_file):
    Path(output_file).parent.mkdir(parents=True, exist_ok=True)
    
    # Configurar leitor do bag com rosbag2_py
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    with open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['stamp_sec', 'stamp_nsec', 'action', 'src_id', 'dest_id', 'relative_rad'])
        
        while reader.has_next():
            topic_name, data, timestamp_ns = reader.read_next()
            if topic_name == topic:
                try:
                    # Usar deserialização para TopologicalAction
                    msg = deserialize_message(data, TopologicalAction)
                    
                    # Extrair dados da mensagem deserializada
                    stamp_sec = msg.header.stamp.sec
                    stamp_nsec = msg.header.stamp.nanosec
                    action = msg.action
                    src_id = msg.src_id
                    dest_id = msg.dest_id
                    relative_rad = msg.relative_rad
                    
                    writer.writerow([stamp_sec, stamp_nsec, action, src_id, dest_id, relative_rad])
                    
                except Exception as e:
                    print(f"Erro processando mensagem TopologicalAction: {str(e)[:200]}")
                    
def extract_RobotPose(bag_path, topic, output_file):
    Path(output_file).parent.mkdir(parents=True, exist_ok=True)
    
    # Configurar leitor do bag com rosbag2_py
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    with open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['stamp_sec', 'stamp_nsec', 'pos_x', 'pos_y', 'pos_z', 'ori_x', 'ori_y', 'ori_z', 'ori_w'])
        
        while reader.has_next():
            topic_name, data, timestamp_ns = reader.read_next()
            if topic_name == topic:
                try:
                    # Usar deserialização para PoseStamped (método mais robusto)
                    msg = deserialize_message(data, PoseStamped)
                    
                    # Extrair dados da mensagem deserializada
                    stamp_sec = msg.header.stamp.sec
                    stamp_nsec = msg.header.stamp.nanosec
                    pos_x = msg.pose.position.x
                    pos_y = msg.pose.position.y
                    pos_z = msg.pose.position.z
                    ori_x = msg.pose.orientation.x
                    ori_y = msg.pose.orientation.y
                    ori_z = msg.pose.orientation.z
                    ori_w = msg.pose.orientation.w
                    
                    writer.writerow([stamp_sec, stamp_nsec, pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w])
                    
                except Exception as e:
                    print(f"Erro processando mensagem RobotPose: {str(e)[:200]}")

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Extract messages from LocalView, TopologicalAction and RobotPose to CSV')
    parser.add_argument('bag_file', help='Path to folder that contains bag in .db3 format')
    parser.add_argument('--topic_root', default='surveyor', help='Topic root name to extract from bag')
    parser.add_argument('--output_path', default='exported_data', help='output path for CSV files')

    args = parser.parse_args()
    view_template_topic = '/'+args.topic_root+'/view_template'
    view_template_output = args.output_path+'/vt_id.csv'

    topological_action_topic = '/'+args.topic_root+'/PoseCell/TopologicalAction'
    topological_action_output = args.output_path+'/em_id.csv'

    robot_pose_topic = '/'+args.topic_root+'/ExperienceMap/RobotPose'
    robot_pose_output = args.output_path+'/pose.csv'

    try:
        print("⏳ Iniciando extração de dados ViewTemplate...")
        extract_ViewTemplate(
            args.bag_file,
            view_template_topic,
            view_template_output
        )
        print("✅ Extraction completed successfully!")
        print("📊 Generated file statistics:")
        print(f"   - Length: {Path(view_template_output).stat().st_size/1024:.2f} KB")
        print(f"   - Lines: {sum(1 for _ in open(view_template_output))}")

        print("⏳ Iniciando extração de dados TopologicalAction...")
        extract_TopologicalAction(
            args.bag_file,
            topological_action_topic,
            topological_action_output
        )
        print("✅ Extração concluída com sucesso!")
        print("📊 Estatísticas do arquivo gerado:")
        print(f"   - Tamanho: {Path(topological_action_output).stat().st_size/1024:.2f} KB")
        print(f"   - Linhas: {sum(1 for _ in open(topological_action_output))}")

        print("⏳ Iniciando extração de dados RobotPose...")
        extract_RobotPose(
            args.bag_file,
            robot_pose_topic,
            robot_pose_output
        )
        print("✅ Extração concluída com sucesso!")
        print("📊 Estatísticas do arquivo gerado:")
        print(f"   - Tamanho: {Path(robot_pose_output).stat().st_size/1024:.2f} KB")
        print(f"   - Linhas: {sum(1 for _ in open(robot_pose_output))}")
    except Exception as e:
        print(f"❌ Erro fatal: {str(e)}")