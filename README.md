<p align="center">
  <h2 align="center">Odin Navigation Stack</h2>
</p>

<div align="center">
  <a href="https://ManifoldTechLtd.github.io/Odin-Nav-Stack-Webpage">
  <img src='https://img.shields.io/badge/Webpage-OdinNavStack-blue' alt='webpage'></a>  
  <a href="https://www.apache.org/licenses/LICENSE-2.0">
  <img src='https://img.shields.io/badge/License-Apache2.0-green' alt='Apache2.0'></a>  
  <a href="https://www.youtube.com/watch?v=du038MPxc0s">
  <img src='https://img.shields.io/badge/Video-YouTube-red' alt='youtube'></a>  
  <a href="https://www.bilibili.com/video/BV1sFBXBmEum/">
  <img src='https://img.shields.io/badge/Video-bilibili-pink' alt='bilibili'></a>  
  <a href="https://wiki.ros.org/noetic">
  <img src='https://img.shields.io/badge/ROS-Noetic-orange' alt='noetic'></a>
</div>

**Odin1** is a high-performance spatial sensing module that delivers **high-precision 3D mapping**, **robust relocalization**, and rich sensory streams including **RGB images**, **depth**, **IMU**, **odometry**, and **dense point clouds**. Built on this foundation, we have developed various robotic intelligence stacks for ground platforms like the **Unitree Go2**, enabling:

- **Autonomous navigation with high-efficiency dynamic obstacle avoidance**  
- **Semantic object detection + natural-language navigation**  
- **Vision-Language Model (VLM) scene understanding and description**  

## Key Features

- **High-Accuracy SLAM & Persistent Relocalization** (inside Odin1, not open-sourced)  
  Real-time mapping with long-term relocalization using compact binary maps.
- **Dynamic Obstacle-Aware Navigation** (fully open-sourced)  
  Reactive local planners combined with global path planning for safe, smooth motion in complicated environments.
- **Semantic Navigation** (fully open-sourced)  
  Detect, localize, and navigate to objects using spoken or typed commands (e.g., _“Go to the left of the chair”_).
- **Vision-Language Integration** (fully open-sourced)  
  Generate contextual scene descriptions in natural language using multimodal AI.
- **Modular, ROS1-Based Architecture**  
  Easy to extend, customize, and integrate into your own robotic applications.

# Quick Start

The code has been tested on:
- OS: Ubuntu 20.04  
- ROS: ROS1 Noetic  
- Robot Platform: Unitree Go2  
- Hardware: NVIDIA Jetson (Orin Nano) or x86 with GPU


## 1. Clone the Repository

``` shell
git clone --depth 1 --recursive https://github.com/ManifoldTechLtd/Odin-Nav-Stack.git
```

## 2. Install System Dependencies
``` shell
export ROS_DISTRO=noetic
sudo apt update
sudo apt install -y \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-tf2-eigen \
    ros-${ROS_DISTRO}-pcl-ros
```

## 3. Install Unitree Go2 SDK
Follow the official guide:
[Unitree Go2 SDK](https://support.unitree.com/home/zh/developer/Obtain%20SDK?spm=a2ty_o01.29997173.0.0.737bc921PvkEw8)

## 4. Set Up Conda & Mamba 
Follow the installation in [miniconda](https://www.anaconda.com/docs/getting-started/miniconda/install#basic-install-instructions).
``` shell
conda install -n base -c conda-forge mamba
# Re-login to your shell after installation
```

## 5. Create the NeuPAN Environment 
``` shell
export ROS_DISTRO=noetic
mamba create -n neupan -y
conda activate neupan
conda config --env --add channels conda-forge
conda config --env --remove channels defaults
conda config --env --add channels robostack-${ROS_DISTRO}

mamba install -n neupan ros-${ROS_DISTRO}-desktop colcon-common-extensions catkin_tools rosdep -y
mamba run -n neupan pip install torch==2.8.0 --index-url https://download.pytorch.org/whl/cpu
```

**For different Jetson users**: Replace the PyTorch install with a compatible .whl from [NVIDIA's Jeston PyTorch Page](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048).

## 6. Build the ROS Workspace 
``` shell
cd ros_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
```

## 7. Set USB Rules for Odin1

``` shell
sudo vim /etc/udev/rules.d/99-odin-usb.rules
```

Add the following content to `/etc/udev/rules.d/99-odin-usb.rules`:

``` shell
SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0019", MODE="0666", GROUP="plugdev"
```

Reload rules and reinsert devices
``` shell
sudo udevadm control --reload
sudo udevadm trigger
```

# Usage

- Mapping & Relocalization
- Navigation
- YOLO object detection
- VLM scene explanation

## Mapping and Relocalization with Odin1

Building maps and performing relocalization with Odin1

### 1. Configure Mapping Mode
Edit `ros_ws/src/odin_ros_driver/config/control_command.yaml`, set `custom_map_mode: 1` to enable mapping.

### 2. Launch Mapping 

Terminal 1 – Start Odin driver:
``` shell
source ros_ws/devel/setup.bash
roslaunch odin_ros_driver odin1_ros1.launch
```

Terminal 2 – Run mapping script:
``` shell
bash scripts/map_recording.sh awesome_map
```

The pcd map will be saved to `ros_ws/src/pcd2pgm/maps/` and the grid map will be saved to `ros_ws/src/map_planner/maps/`

After the map is constructed, you can view and modify the grid map using GIMP:
``` shell
sudo apt update && sudo apt install gimp
```

### 3. Relocalization & Navigation
Enable relocalization by editing `control_command.yaml`: 
``` shell
custom_map_mode: 2
relocalization_map_abs_path: "/abs/path/to/your/map"
```

Launch: 
``` shell
roslaunch odin_ros_driver odin1_ros1.launch
```

Verify TF tree: 
``` shell
rostopic hz /tf
# or visualize TF tree in rqt: map → odom → odin1_base_link
```
Note: Relocalization may require manually initiating motion.

## Navigation Modes
### Standard ROS Navigation
Use Nav1 and move-base. Please [install](https://wiki.ros.org/navigation) before running.
``` shell
roslaunch navigation_planner navigation_planner.launch
```

### Custom Planner
Tune `global_planner.yaml` and `local_planner.yaml` in `ros_ws/src/model_planner`, then:
``` shell
roslaunch model_planner model_planner.launch
```
You can modify the code and replace it with your own custom algorithm.

### End-to-End Neural Planner
This is our recommended high-performance local planner; please refer to the paper: [NeuPAN](https://ieeexplore.ieee.org/document/10938329/).

``` shell
# Terminal 1
roslaunch map_planner whole.launch

# Terminal 2
mamba activate neupan
python NeuPAN/neupan/ros/neupan_ros.py
```
Use RViz to publish 2D Nav Goals.

## Object detection

Enables navigation to specific objects. Requires depth maps and undistorted images from Odin1.

### 1. Install YOLOv5 in Virtual Environment:

First, install YOLOv5:
``` shell
python3 -m venv ros_ws/venvs/ros_yolo_py38
source ros_ws/venvs/ros_yolo_py38/bin/activate
pip install --upgrade pip "numpy<2.0.0"
cd ros_ws/src && git clone https://github.com/ultralytics/yolov5.git
pip install -r yolov5/requirements.txt
```
Please note that we encountered a conflict between the automatic installation of torch and torchvision on a certain Jetson Orin Nano. If you encounter this issue, please refer to the troubleshooting section.

Then, install other dependencies:
``` shell
pip install opencv-python pillow pyyaml requests tqdm scipy matplotlib seaborn pandas empy==3.3.4 catkin_pkg ros_pkg vosk sounddevice
```

Verify PyTorch and CUDA:
``` shell
python -c "import torch, torchvision; print('PyTorch:', torch.__version__); print('torchvision:', torchvision.__version__); print('CUDA available:', torch.cuda.is_available())"
```

Download resources:
``` shell
mkdir -p ros_ws/src/yolo_ros/scripts/voicemodel
cd ros_ws/src/yolo_ros/scripts/voicemodel
wget https://alphacephei.com/vosk/models/vosk-model-small-cn-0.22.zip
unzip vosk-model-small-cn-0.22.zip
wget https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.pt -O ../models/yolov5s.pt
chmod +x ../yolo_detector.py
```


### 2. Calibrate Camera 
Copy `Tcl_0` and `cam_0` from `odin_ros_driver/config/calib.yaml` into `yolo_detector.py`. 

### 3. Launch 
Terminal 1: 
``` shell
roslaunch odin_ros_driver odin1_ros1.launch
```

Terminal 2: 
``` shell
./run_yolo_detector.sh
```

In Terminal 2, you can enter the following commands to control it:
- list: Query recognized objects.
- object name: Display the 3D position in RViz.
- Move to the [Nth] [object] [direction]: Publish a navigation goal. (Supprot Chinese input)
- mode: Toggle between text and voice input.


## Vision-Language Model (VLM)
Install LLaMA.cpp:
``` shell
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
brew install llama.cpp
```

Download models (e.g., SmolVLM):
``` shell
wget https://huggingface.co/ggml-org/SmolVLM-500M-Instruct-GGUF/resolve/main/SmolVLM-500M-Instruct-Q8_0.gguf
wget https://huggingface.co/ggml-org/SmolVLM-500M-Instruct-GGUF/resolve/main/mmproj-SmolVLM-500M-Instruct-Q8_0.gguf
```

### Launch

Terminal 1: 
``` shell
llama-server -m SmolVLM-500M-Instruct-Q8_0.gguf --mmproj mmproj-SmolVLM-500M-Instruct-Q8_0.gguf
```

Terminal 2:
``` shell
roslaunch odin_ros_driver odin1_ros1.launch
```

Terminal 3:
``` shell
roslaunch odin_vlm_terminal odin_vlm_terminal.launch
```

## VLN

### Launch

Terminal 1: 
``` shell
roslaunch map_planner whole.launch
```

Terminal 2:
``` shell
roslaunch odin_ros_driver odin1_ros1.launch
```

Terminal 3:
``` shell
mamba activate neupan
python NeuPAN/neupan/ros/neupan_ros.py
```

Terminal 4:
``` shell
mamba activate neupan
python scripts/str_cmd_control.py
```

Terminal 5:
``` shell
mamba activate neupan
python scripts/VLN.py
```

# Troubleshooting

## torch conflict with torchvision
Error:`torch.cuda.is_available() returns False`

Cause: torchvision overwrote the CUDA-enabled PyTorch installation.

Fix:
``` shell
pip uninstall torch torchvision torchaudio
# Reinstall torch from .whl
pip install torch-*.whl
pip install --no-cache-dir "git+https://github.com/pytorch/vision.git@v0.16.0"
```
If the problem persists, you can try the following methods:
Navigate to `cd ros_ws/src/yolov5/utils`, open the `general.py` file, and locate the following code:
``` python
# Batched NMS
c = x[:, 5:6] * (0 if agnostic else max_wh)  # classes
boxes, scores = x[:, :4] + c, x[:, 4]  # boxes (offset by class), scores
i = torchvision.ops.nms(boxes, scores, iou_thres)  # NMS
```
Modify the YOLO code:
``` python
# Batched NMS (using pure PyTorch to avoid torchvision.ops compatibility issues)
c = x[:, 5:6] * (0 if agnostic else max_wh)  # classes
boxes, scores = x[:, :4] + c, x[:, 4]  # boxes (offset by class), scores
# Pure PyTorch NMS implementation
sorted_idx = torch.argsort(scores, descending=True)
keep = []
while len(sorted_idx) > 0:
    current_idx = sorted_idx[0]
    keep.append(current_idx)
    if len(sorted_idx) == 1:
        break
    current_box = boxes[current_idx:current_idx+1]
    rest_boxes = boxes[sorted_idx[1:]]
    # Calculate IoU
    inter_x1 = torch.max(current_box[:, 0], rest_boxes[:, 0])
    inter_y1 = torch.max(current_box[:, 1], rest_boxes[:, 1])
    inter_x2 = torch.min(current_box[:, 2], rest_boxes[:, 2])
    inter_y2 = torch.min(current_box[:, 3], rest_boxes[:, 3])
    inter_w = torch.clamp(inter_x2 - inter_x1, min=0)
    inter_h = torch.clamp(inter_y2 - inter_y1, min=0)
    inter_area = inter_w * inter_h
    current_area = (current_box[:, 2] - current_box[:, 0]) * (current_box[:, 3] - current_box[:, 1])
    rest_area = (rest_boxes[:, 2] - rest_boxes[:, 0]) * (rest_boxes[:, 3] - rest_boxes[:, 1])
    union_area = current_area + rest_area - inter_area
    iou = inter_area / union_area
    sorted_idx = sorted_idx[1:][iou < iou_thres]
i = torch.tensor(keep, dtype=torch.long, device=boxes.device)
``` 
 
## libgomp problem
Error: `libgomp` not found or similar problem

Cause: Missing installation or corrupted library files.

Fix:
``` shell
for f in ~/venvs/ros_yolo_py38/lib/python3.8/site-packages/torch.libs/libgomp*.so*; do
    [ -f "$f" ] && mv "$f" "$f.bak"
done
```

# Acknowledgements

Thanks to the excellent work by [ROS Navigation](https://github.com/ros-planning/navigation), [NeuPAN](https://github.com/hanruihua/NeuPAN), [Ultralytics YOLO](https://github.com/ultralytics/ultralytics) and [Qwen](https://github.com/QwenLM/Qwen3-VL).

Special thanks to [hanruihua](https://github.com/hanruihua), [KevinLADLee](https://github.com/KevinLADLee) and [bearswang](https://github.com/bearswang) for their technical support.

