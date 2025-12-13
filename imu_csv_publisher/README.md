# IMU CSV Publisher for ROS2

ROS2 package for publishing IMU data from iPhone to ROS2.

## Structure

- `scripts/` - iPhone data collection scripts
- `imu_csv_publisher/` - ROS2 Python package
- `launch/` - ROS2 launch files

## Usage

1. Collect data from iPhone using scripts in `scripts/` folder
2. Convert to ROS2 format
3. Publish using ROS2 node

## Branches

- `main` - ROS2 package only
- `with-scripts` - ROS2 package + iPhone scripts