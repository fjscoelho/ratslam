#!/usr/bin/env python3
import rclpy
from rclpy.serialization import deserialize_message
from topological_msgs.msg import TopologicalMap
import sqlite3
import csv
from datetime import datetime
import argparse

# comand example: python3 extract_nodes_edges_map.py output_bags/surveyor_test1.bag/surveyor_test1.bag_0.db3 --topic_root surveyor --output_path exported_data

def extract_topological_data(bag_file, topic_name, nodes_csv, links_csv):
    """Extrai nós e conexões para CSVs separados"""
    conn = sqlite3.connect(f"file:{bag_file}?mode=ro", uri=True)
    cursor = conn.cursor()

    try:
        # write node file
        with open(nodes_csv, 'w', newline='') as nodes_file:
            node_writer = csv.writer(nodes_file)
            node_writer.writerow(['node_count', 'edge_count',
                'stamp_sec', 'stamp_nsec', 'id', 'x', 'y', 'z',
                'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w'
            ])

            # write link file
            with open(links_csv, 'w', newline='') as links_file:
                link_writer = csv.writer(links_file)
                link_writer.writerow([
                    'id', 'source_id', 'destination_id',
                    'duration_sec', 'duration_nanosec'
                ])

                # get all messages from topic
                cursor.execute("""
                    SELECT m.timestamp, m.data 
                    FROM messages m 
                    JOIN topics t ON m.topic_id = t.id 
                    WHERE t.name = ?
                    ORDER BY m.timestamp
                    """, (topic_name,))

                for ts, data in cursor:
                    try:
                        msg = deserialize_message(data, TopologicalMap)
                        # timestamp = (ts/1e9) # datetime.fromtimestamp(ts/1e9).isoformat()
                        stamp_sec = msg.header.stamp.sec
                        stamp_nsec = msg.header.stamp.nanosec
                        node_count = msg.node_count
                        edge_count = msg.edge_count
                        # Process nodes
                        for node in msg.node:
                            node_writer.writerow([
                                node_count, edge_count,
                                node.pose.header.stamp.sec,
                                node.pose.header.stamp.nanosec,
                                node.id,
                                node.pose.pose.position.x,
                                node.pose.pose.position.y,
                                node.pose.pose.position.z,
                                node.pose.pose.orientation.x,
                                node.pose.pose.orientation.y,
                                node.pose.pose.orientation.z,
                                node.pose.pose.orientation.w,
                            ])

                        # Process edges
                        for edge in msg.edge:
                            link_writer.writerow([
                                edge.id,
                                edge.source_id,
                                edge.destination_id,
                                edge.duration.sec,
                                edge.duration.nanosec,
                                # timestamp
                            ])

                    except Exception as e:
                        print(f"Processing messsage error: {e}")

        print(f"Data saved in:\n- Nodes: {nodes_csv}\n- Links: {links_csv}")

    finally:
        conn.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Extrai dados de mapa topológico para CSV')
    parser.add_argument('bag_file', help='Arquivo .db3 de entrada')
    parser.add_argument('--topic_root', default='surveyor', help='Topic root name to extract from bag')
    parser.add_argument('--output_path', default='exported_data', help='output path for CSV files')
    
    args = parser.parse_args()
    topic_name = '/'+args.topic_root+'/ExperienceMap/Map'
    nodes_output = args.output_path+'/nodes.csv' # output file path for nodes
    links_output = args.output_path+'/links.csv' # output file path for links

    # ROS 2 initialize to desserialization
    rclpy.init()
    extract_topological_data(args.bag_file, topic_name, nodes_output, links_output)
    rclpy.shutdown()