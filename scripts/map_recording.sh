#!/bin/bash

# Copyright (c) 2025 Manifold Tech Ltd.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


# Usage: ./map_recording.bash [FILENAME] [VOXEL_SIZE] [APPLY_FILTER]

FILENAME="${1:-mymap}"  # Filename
VOXEL_SIZE="${2:-0.05}"  # Voxel size in meters
APPLY_FILTER="${3:-true}"  # Whether to apply statistical filter

echo "Map will be saved as: ${FILENAME}.pcd"
echo "Voxel size: ${VOXEL_SIZE} m"
echo "Apply statistical filter: ${APPLY_FILTER}"

set -e

pcd2pgm_pid=""
pointcloud_saver_pid=""

cleanup() {
  for pid in "$pcd2pgm_pid" "$pointcloud_saver_pid"; do
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
  pcd2pgm_pid=""
  pointcloud_saver_pid=""
}

trap cleanup EXIT

if [ ! -d "ros_ws" ]; then
  echo "ros_ws folder does not exist. Please make sure you are in the correct directory."
  exit 1
fi

if [ ! -d "ros_ws/devel" ]; then
  echo "ros_ws/devel folder does not exist. Please build the workspace first."
  exit 1
fi

odin_config_file="ros_ws/src/odin_ros_driver/config/control_command.yaml"
if [ ! -f "$odin_config_file" ]; then
  echo "Odin config file does not exist at $odin_config_file. Please check the path."
  exit 1
fi
mode=$(sed -n 's/^[[:space:]]*custom_map_mode:[[:space:]]*\([0-9]\+\).*$/\1/p' "$odin_config_file" | head -n1)
if [ -z "$mode" ]; then
  echo "Cannot find parameter 'custom_map_mode' in $odin_config_file. Exiting."
  exit 1
fi
case "$mode" in
  1)
    ;;
  *)
    echo "Odin running at custom_map_mode: $mode, which is not mapping mode."
    echo "Please set 'custom_map_mode' to 1 in $odin_config_file and restart the Odin ROS driver."
    exit 1
    ;;
esac

source ros_ws/devel/setup.bash

while true; do
  read -r -p "Press [Enter] to start recording the map: " input
  if [ -z "$input" ]; then
    break
  elif [ "$input" = "q" ]; then
    echo "Recording cancelled by user."
    exit 0
  else
    echo "Unknow input. Please press [Enter] to start recording or 'q' to quit."
  fi
done

echo "Starting pointcloud_saver launch ..."
roslaunch pointcloud_saver pointcloud_saver.launch &
pointcloud_saver_pid=$!

sleep 3

rosservice call /pointcloud_saver/start_recording "{}"
echo "Start recording map..."

while true; do
  read -r -p "Press [Enter] to stop recording the map: " input
  if [ -z "$input" ]; then
    break
  elif [ "$input" = "q" ]; then
    echo "Stop recording map..."
    rosservice call /pointcloud_saver/stop_recording "{}"
    echo "User exit."
    exit 0
  else
    echo "Unknow input. Please press [Enter] to stop recording or 'q' to quit."
  fi
done

echo "Stop recording map..."
rosservice call /pointcloud_saver/stop_recording "{}"
bash ros_ws/src/odin_ros_driver/set_param.sh save_map 1

sleep 3

echo "============================================"
echo "Saving PCD Map with the following parameters:"
echo "============================================"
echo "File name: ${FILENAME}"
echo "Voxel size: ${VOXEL_SIZE} m"
echo "Filter: ${APPLY_FILTER}"
echo "============================================"

rosservice call /pointcloud_saver/save_map \
  "filename: '${FILENAME}'
voxel_size: ${VOXEL_SIZE}
apply_statistical_filter: ${APPLY_FILTER}"

if [ -n "$pointcloud_saver_pid" ]; then
  kill -INT "$pointcloud_saver_pid" 2>/dev/null || true
  wait "$pointcloud_saver_pid" 2>/dev/null || true
  pointcloud_saver_pid=""
fi

echo "Save PCD map success."
echo "Converting PCD map to grid ..."

roslaunch pcd2pgm pcd2pgm.launch default_map_filename:="${FILENAME}.pcd" &
pcd2pgm_pid=$!

sleep 5
rosrun map_server map_saver -f ros_ws/src/map_planner/maps/${FILENAME}
sed -i "s|image: ros_ws/src/map_planner/maps/${FILENAME}.pgm|image: ${FILENAME}.pgm|" ros_ws/src/map_planner/maps/${FILENAME}.yaml
echo "Save grid map success."

cleanup

read -r -p "Press [Enter] to change config files for current map, other to skip: " input
if [ -z "$input" ]; then
  sed -i 's/custom_map_mode:[[:space:]]*1/custom_map_mode: 2/g' $odin_config_file
  echo "Changed 'custom_map_mode' to 2 (Relocalization mode) in $odin_config_file."
  sed -i '/<arg name="map_file_name"/c\  <arg name="map_file_name" default="'"$FILENAME"'" />' ros_ws/src/map_planner/launch/whole.launch
  echo "Set map_file_name to ${FILENAME} in ros_ws/src/map_planner/launch/whole.launch."
else
  echo "Skipped changing config files."
  echo "You may need to manually change 'custom_map_mode' to 2 and in $odin_config_file and set map_file_name in ros_ws/src/map_planner/launch/whole.launch."
fi

echo "You need to change relocalization_map_abs_path in $odin_config_file before relocalization and restart odin driver."
echo "Map recording and saving process completed."

