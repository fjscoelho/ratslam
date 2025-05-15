#!/usr/bin/env python3
import rclpy
from rclpy.serialization import deserialize_message
from topological_msgs.msg import TopologicalMap
import sqlite3
import csv
from datetime import datetime

def extract_topological_map(bag_file, output_csv):
    # Conecta ao banco de dados SQLite do bag
    conn = sqlite3.connect(f"file:{bag_file}?mode=ro", uri=True)
    cursor = conn.cursor()

    # Encontra o ID do tópico
    cursor.execute("SELECT id FROM topics WHERE name='/surveyor/ExperienceMap/Map'")
    topic_id = cursor.fetchone()[0]

    # Prepara o arquivo CSV de saída
    with open(output_csv, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        
        # Escreve cabeçalho (ajuste conforme sua estrutura de mensagem)
        header = [
            "timestamp",
            "node_id", "node_x", "node_y", "node_z",
            "edge_id", "source_id", "destination_id"
        ]
        writer.writerow(header)

        # Extrai todas as mensagens do tópico
        cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id=?", (topic_id,))
        for row in cursor:
            try:
                # Desserializa a mensagem: row[1] → Contém os dados binários serializados da mensagem
                msg = deserialize_message(row[1], TopologicalMap)
                
                # Processa nós
                for node in msg.node:
                    writer.writerow([
                        # row[0] → Contém o timestamp (não usado na desserialização)
                        datetime.fromtimestamp(row[0]/1e9).isoformat(),  # timestamp
                        node.id, node.pose.position.x, node.pose.position.y, node.pose.position.z,
                        "", "", ""  # Placeholders para arestas
                    ])
                
                # Processa arestas
                for edge in msg.edge:
                    writer.writerow([
                        datetime.fromtimestamp(row[0]/1e9).isoformat(),  # timestamp
                        "", "", "", "",  # Placeholders para nós
                        edge.id, edge.source_id, edge.destination_id
                    ])
            
            except Exception as e:
                print(f"Erro ao processar mensagem: {e}")

    conn.close()
    print(f"Dados salvos em {output_csv}")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_file", help="Caminho para o arquivo .db3")
    parser.add_argument("output_csv", help="Caminho para o arquivo CSV de saída")
    args = parser.parse_args()

    # Inicializa ROS 2 para desserialização
    rclpy.init()
    extract_topological_map(args.bag_file, args.output_csv)
    rclpy.shutdown()