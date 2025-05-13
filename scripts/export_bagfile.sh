#!/bin/bash

while getopts f:t: option
do
    case "${option}"
    in
        f) BAG_FILE=${OPTARG};;
        t) TOPIC_ROOT=${OPTARG};;
    esac
done

# Remove barras do prefixo do tópico para nomes de arquivo
TOPIC_PREFIX=$(echo "$TOPIC_ROOT" | sed 's/\///g')

# Exporta para CSV
ros2 bag export "$BAG_FILE" --output-dir temp_export

# Renomeia arquivos para os nomes esperados
mv "temp_export/${TOPIC_PREFIX}_LocalView_Template.csv" vt_id.dat
mv "temp_export/${TOPIC_PREFIX}_PoseCell_TopologicalAction.csv" em_id.dat
mv "temp_export/${TOPIC_PREFIX}_ExperienceMap_RobotPose.csv" pose.dat
mv "temp_export/${TOPIC_PREFIX}_ExperienceMap_Map.csv" map.dat

# Limpa a pasta temporária (opcional)
rm -r temp_export

echo "Dados extraídos: vt_id.dat, em_id.dat, pose.dat, map.dat"