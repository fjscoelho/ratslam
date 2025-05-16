# RatSLAM Scripts

This folder contains utility scripts for post processing data in the RatSLAM pipeline.

## Suggested Folder Structure
```
ratslam/
├── 📁 scripts/                  # Main script location
│   ├── 📄 extract_gps_data.py   # Extract GPS to csv
│   ├── 📄 extract_topics.py     # Extract topological messages
│   ├── 📄 extract_nodes_edges_map.py
│   ├── 📄 show_deadereckoning.m # Matlab odometry plots
│   ├── 📄 lat_long_to_cartesian.m
│   ├── 📄 show_em.py            # Experience map evolution
│   ├── 📄 show_id.py            # Template visualization
│   ├── 📁 exported_data/        # CSV files
│   └── 📁 output_bags/          # Processed ROS bags
│       └── 📁 bag_dataset1/
│           ├── 📄 metadata.yaml
│           └── 📄 bag_dataset1.db3
│   └──📁 Figures/                  # Visualization outputs
        ├── 🖼️ Final_Exp_map.png
        ├── 🖼️ Final_Exp_map.eps
        ├── 🎞️ experience_map_evolution.avi
        └── 📁 map_evolution/        # Partial evolution frames
```                        

### Core Requirements
- Python 3.8+
- pip package manager

### ROS 2 Dependencies
- ROS 2 Humble or Rolling (recommended)
- `rosbag2` package:
  ```bash
  sudo apt-get install ros-$ROS_DISTRO-rosbag2

## Scripts Overview

### `extract_gps_data.py`
Extracts **GPS data** from ROS 2 bag files and exports it to CSV format.


#### Usage:
```bash
python3 extract_gps_data.py <path_to_ros2_bag> --topic <gps_topic_name> --gps_data <output_csv_path>
```

#### Parameters:
| Argument          | Type    | Default      | Description                                           |
|-------------------|---------|--------------|-------------------------------------------------------|
| `input_path`      | string  | -            | Path to ROS 2 bag folder (must contain metadata.yaml) |
| `--topic`, `-t`   | string  | `/gps/fix`   | ROS topic to process                                  |
| `output_path`     | string  | `gps.csv`    | output file for gps data CSV                          |

#### Example:

```bash
python3 extract_gps_data.py ../../../bags/20250416_131706_bag_ros2 --topic /surveyor/gps_fix --gps_data exported_data/gps.csv
```

### `extract_nodes_edges.py`
Extracts **experience map data** from ROS 2 bag files and exports it to CSV format.


#### Usage:
```bash
python3 extract_nodes_edges.py <path_to_ros2_bag> --topic_root <topic_root_name> --output_path <output_path>
```

#### Parameters:
| Argument          | Type    | Default         | Description                                                  |
|-------------------|---------|-----------------|--------------------------------------------------------------|
| `input_file`      | string  | -               | Path to ROS 2 bag folder (in this case include /bagfile.db3) |
| `topic_root`      | string  | `surveyor`      | topic root name                                              |
| `output_path`     | string  | `exported_data` | output file for links data CSV                               |

#### Example:

```bash
python3 python3 extract_nodes_edges_map.py output_bags/surveyor_test1.bag/surveyor_test1.bag_0.db3 --topic_root surveyor --output_path exported_data
```

### `extract_topics.py`
Extracts **LocalView, TopologicalAction and RobotPose data** from ROS 2 bag files and exports it to CSV format.

#### Usage:
```bash
python3 extract_topics.py <path_to_ros2_bag> --topic_root <topic_root_name> --output_path <output_path>
```

#### Parameters:
| Argument          | Type    | Default         | Description                                           |
|-------------------|---------|-----------------|-------------------------------------------------------|
| `input_path`      | string  | -               | Path to ROS 2 bag folder (must contain metadata.yaml) |
| `topic_root`      | string  | `surveyor`      | topic root name                                       |
| `output_path`     | string  | `exported_data` | output file for links data CSV                        |

#### Example:

```bash
python3 python3 extract_topics.py output_bags/surveyor_test1.bag --topic_root surveyor --output_path exported_data
```


