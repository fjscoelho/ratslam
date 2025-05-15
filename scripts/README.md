# RatSLAM Scripts

This folder contains utility scripts for processing data in the RatSLAM pipeline.

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
Extracts GPS data from ROS 2 bag files and exports it to CSV format.


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




