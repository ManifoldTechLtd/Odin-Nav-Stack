# Docker Setup

Run the **Odin Navigation Stack** (Odin1 driver + NeuPAN local planner + global A\* + dynamic obstacle memory) from a container, without installing ROS or PyTorch on the host.

The container produces `/cmd_vel` on a host-network ROS1 master. YOLO, VLM, and VLN are **not** included; deploy those on a companion machine if needed.

## Prerequisites

- **Hardware**: NVIDIA Jetson (Orin Nano / Orin AGX) running JetPack 5.1.x (L4T R35.x). Other Jetson generations require a different `BASE_IMAGE`; see the note below.
- **Docker**: 24.0 or newer.
- **Docker Compose**: v2 (the `docker compose` plugin).
- **Odin1 USB rule** on the host (`/etc/udev/rules.d/99-odin-usb.rules`):
  ```
  SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0019", MODE="0666", GROUP="plugdev"
  ```
  After creating it: `sudo udevadm control --reload && sudo udevadm trigger`.

> **Other Jetson versions**: For JetPack 6.x (L4T R36.x), override the base:
> `docker build -f docker/Dockerfile.global --build-arg BASE_IMAGE=dustynv/ros:noetic-pytorch-l4t-r36.2.0 -t odin-nav:latest .`

## Build

From the **repo root**:

```bash
# Global apt/pip sources
docker build -f docker/Dockerfile.global -t odin-nav:latest .

# Mainland China mirrors (Tsinghua)
docker build -f docker/Dockerfile.cn -t odin-nav:latest .
```

Or use compose:

```bash
docker compose -f docker/docker-compose.yml build
# To pick the .cn dockerfile:
DOCKERFILE=docker/Dockerfile.cn docker compose -f docker/docker-compose.yml build
```

## First-run setup: configuration overrides

The compose file mounts four config files from `docker/configs/` over the in-image defaults. Copy the defaults out before the first `up`:

```bash
mkdir -p docker/configs docker/maps
cp ros_ws/src/odin_ros_driver/config/control_command.yaml  docker/configs/
cp NeuPAN/neupan/ros/configs/config.yaml                   docker/configs/
cp NeuPAN/neupan/ros/configs/planner.yaml                  docker/configs/
cp ros_ws/src/map_planner/launch/whole.launch              docker/configs/
```

Now you can edit any of those four files on the host and the next `up` picks them up — **no rebuild required**.

## Run

```bash
docker compose -f docker/docker-compose.yml up
```

The container starts:
- `roslaunch map_planner whole.launch` (driver, A\*, fake360, goal state machine)
- `python NeuPAN/neupan/ros/neupan_ros.py` (local planner)

ROS1 master runs on `localhost:11311` on the host (network_mode: host). On the host you can:

```bash
source /opt/ros/noetic/setup.bash
rostopic echo /cmd_vel
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped ...
```

To stop:

```bash
docker compose -f docker/docker-compose.yml down
```

## Mount summary

| Host path | Container path | Purpose |
|---|---|---|
| `docker/configs/control_command.yaml` | `…/odin_ros_driver/config/control_command.yaml` | Driver mode + relocalization map path |
| `docker/configs/config.yaml` | `…/NeuPAN/neupan/ros/configs/config.yaml` | NeuPAN ROS topics, stuck-escape |
| `docker/configs/planner.yaml` | `…/NeuPAN/neupan/ros/configs/planner.yaml` | NeuPAN MPC weights |
| `docker/configs/whole.launch` | `…/map_planner/launch/whole.launch` | A\* / fake360 / state-machine args |
| `docker/maps/grid/` | `…/map_planner/maps/` | Grid maps (`.pgm` + `.yaml`) |
| `docker/maps/relocalization/` | `/opt/odin/maps/relocalization/` | Odin driver SLAM output (`.bin`) |
| `docker/maps/pcd/` | `…/pcd2pgm/maps/` | Raw point clouds (`.pcd`) |

## Map persistence — where mapping output is saved and how to reuse it

The Odin driver writes a binary relocalization map (`.bin`) every time you run the mapping mode. **By default** (`mapping_result_dest_dir: ""`) it goes to a path inside the container that is **lost when the container is removed**. You must point it at one of the host-mounted directories above.

### Recommended layout

On the host, after `docker compose up` for the first time:

```
docker/maps/
├── grid/                 # auto-populated by map_recording.sh / map_server
│   ├── awesome_map.pgm
│   └── awesome_map.yaml
├── relocalization/       # Odin driver SLAM output (.bin) — persists on host
│   └── awesome_map.bin
└── pcd/                  # raw point clouds
    └── awesome_map.pcd
```

All three subdirectories live on the host filesystem, so a `docker compose down` does not delete them.

### Step-by-step: mapping then relocalization

1. **Edit `docker/configs/control_command.yaml`** for mapping mode and tell the driver to save outputs to the mounted path:
   ```yaml
   custom_map_mode: 1                                 # SLAM/mapping mode
   mapping_result_dest_dir: "/opt/odin/maps/relocalization"
   mapping_result_file_name: "awesome_map"            # the .bin filename stem
   ```

2. **Start the container** and record a map (run `scripts/map_recording.sh` from the host or open a shell into the container — see "Open a shell" below). The driver writes `awesome_map.bin` to `/opt/odin/maps/relocalization/`, which is `docker/maps/relocalization/awesome_map.bin` on the host.

3. **Switch to relocalization mode**. Edit `docker/configs/control_command.yaml`:
   ```yaml
   custom_map_mode: 2                                 # relocalization mode
   relocalization_map_abs_path: "/opt/odin/maps/relocalization/awesome_map.bin"
   ```

4. **`docker compose restart odin-nav`** — the driver loads the `.bin` from the host-mounted path and starts up in relocalization mode against the existing map.

> **Always use the in-container path** (`/opt/odin/maps/...`) inside `control_command.yaml`, **not** a host path like `/home/user/...`. The driver runs inside the container.

### Open a shell to run scripts manually

```bash
docker compose -f docker/docker-compose.yml run --rm odin-nav shell
# inside the container:
cd /opt/odin && bash scripts/map_recording.sh awesome_map
```

## ROS2 bridge

If your robot stack is ROS2, bridge `/cmd_vel` over UDP. Run the snippet below **on the host** (not inside the container):

```bash
# host_ros1, ROS2 side: receive UDP -> publish to /cmd_vel (geometry_msgs/Twist)
# host_ros1, ROS1 side: subscribe /cmd_vel -> forward as UDP datagrams
python3 - <<'PY'
import rospy, socket, struct
from geometry_msgs.msg import Twist
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
def cb(msg):
    sock.sendto(struct.pack('6f',
        msg.linear.x, msg.linear.y, msg.linear.z,
        msg.angular.x, msg.angular.y, msg.angular.z), ('127.0.0.1', 5005))
rospy.init_node('cmd_vel_udp_tx')
rospy.Subscriber('/cmd_vel', Twist, cb)
rospy.spin()
PY
```

ROS2 receiver:

```bash
python3 - <<'PY'
import rclpy, socket, struct
from rclpy.node import Node
from geometry_msgs.msg import Twist
class Rx(Node):
    def __init__(self):
        super().__init__('cmd_vel_udp_rx')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 5005))
        self.create_timer(0.0, self.tick)
    def tick(self):
        data, _ = self.sock.recvfrom(64)
        v = struct.unpack('6f', data)
        m = Twist()
        m.linear.x, m.linear.y, m.linear.z = v[0:3]
        m.angular.x, m.angular.y, m.angular.z = v[3:6]
        self.pub.publish(m)
rclpy.init()
rclpy.spin(Rx())
PY
```

This is intentionally minimal — for production deployments, use `ros1_bridge` or a proper IPC framework.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `failed to pull dustynv/ros:noetic-pytorch-l4t-r35.4.1` | Wrong JetPack/L4T version | Override `--build-arg BASE_IMAGE=...` to match your L4T version |
| Container exits with `cannot find /dev/bus/usb` | Odin1 not plugged in or USB rule missing | Check `lsusb` on host; reload udev rules |
| `roscore` ports already in use | Another ROS master on host | `pkill rosmaster` or change `ROS_MASTER_URI` to a non-default port |
| NeuPAN crashes on startup with `ModuleNotFoundError` | Image was built before the latest NeuPAN submodule update | Pull and rebuild: `git submodule update --remote && docker compose build` |
| `/cmd_vel` is silent | NeuPAN process died (check logs) | `docker logs odin-nav`; usually a missing config file or wrong topic name in `config.yaml` |

## Footprint

Image size on Jetson Orin Nano: ~7 GB (most of it is the base PyTorch + CUDA libs). Build time: ~25-40 minutes on Orin Nano with global sources, ~15-20 minutes with .cn mirrors on a CN connection.
