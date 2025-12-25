'''
Copyright 2025 Manifold Tech Ltd.(www.manifoldtech.com.co)
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
   http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
'''

'''
The script records the robot's current position and orientation in the 'map' frame
and saves these points with user-defined names into JSON and YAML files

Usage:
    mamba activate neupan
    roscore
    python scripts/point_record.py
'''

import rospy
import tf
import os
import json
import yaml
import sys
import time

def save_points(points, directory, filename_base="recorded_points"):
    if not os.path.exists(directory):
        os.makedirs(directory)

    # Save as JSON
    json_path = os.path.join(directory, filename_base + ".json")
    with open(json_path, 'w') as f:
        json.dump(points, f, indent=4)
    rospy.loginfo(f"Saved points to {json_path}")

    # Save as YAML
    yaml_path = os.path.join(directory, filename_base + ".yaml")
    with open(yaml_path, 'w') as f:
        yaml.dump(points, f, default_flow_style=False)
    rospy.loginfo(f"Saved points to {yaml_path}")

def main():
    rospy.init_node('point_recorder')
    listener = tf.TransformListener()

    # Determine save directory: script_dir/../maps
    script_dir = os.path.dirname(os.path.abspath(__file__))
    maps_dir = os.path.join(script_dir, '..', 'maps')
    maps_dir = os.path.abspath(maps_dir)

    points_data = {}
    
    # Load existing points if available to append
    file_name = f"{int(time.time())}"
    json_path = os.path.join(maps_dir, file_name)
    if os.path.exists(json_path):
        try:
            with open(json_path, 'r') as f:
                points_data = json.load(f)
                rospy.loginfo(f"Loaded existing points: {list(points_data.keys())}")
        except Exception as e:
            rospy.logwarn(f"Could not load existing points: {e}")

    rospy.loginfo("Ready to record points. Ensure TF is publishing map -> base_link.")

    while not rospy.is_shutdown():
        try:
            # Python 2/3 compatibility for input
            if sys.version_info[0] < 3:
                point_name = raw_input("Enter point name (or 'q' to quit): ")
            else:
                point_name = input("Enter point name (or 'q' to quit): ")

            if point_name.lower() == 'q':
                break
            
            if not point_name:
                rospy.logwarn("Empty name, skipping.")
                continue

            # Get the transform
            try:
                listener.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(1.0))
                (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                
                point_info = {
                    'position': {
                        'x': trans[0],
                        'y': trans[1],
                        'z': trans[2]
                    },
                    'orientation': {
                        'x': rot[0],
                        'y': rot[1],
                        'z': rot[2],
                        'w': rot[3]
                    }
                }
                
                points_data[point_name] = point_info
                rospy.loginfo(f"Recorded point '{point_name}': {point_info}")
                
                save_points(points_data, maps_dir, file_name)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr(f"TF Error: {e}")

        except EOFError:
            break
        except KeyboardInterrupt:
            break

if __name__ == '__main__':
    main()
