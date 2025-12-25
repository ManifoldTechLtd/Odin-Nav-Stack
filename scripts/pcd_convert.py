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

"""
pcd_convert.py

GPU voxel downsample using Open3D Tensor API, save downsampled point cloud to PLY and GLB (as glTF POINTS).

Usage:
    python pcd_convert.py input.ply --voxel 0.1
"""

import argparse
import sys
import numpy as np
import open3d as o3d
from pathlib import Path
from pygltflib import GLTF2, Scene, Node, Mesh, Primitive, Buffer, BufferView, Accessor, Asset, ARRAY_BUFFER, ELEMENT_ARRAY_BUFFER, FLOAT, VEC3, POINTS

def parse_args():
    p = argparse.ArgumentParser(description="GPU downsample PLY/PCD and save PLY + GLB (point primitives).")
    p.add_argument("input_file_name", help="Input PLY/PCD file")
    p.add_argument("--voxel", type=float, default=0.1, help="Voxel size for downsampling")
    return p.parse_args()

def ensure_cuda():
    try:
        return o3d.core.cuda.is_available()
    except Exception:
        return False

def read_pointcloud_tensor(path, device):
    pcd_t = o3d.t.io.read_point_cloud(path)
    if len(pcd_t.point.positions) == 0:
        raise RuntimeError(f"Loaded point cloud is empty: {path}")
    if pcd_t.device != device:
        pcd_t = pcd_t.to(device)
    return pcd_t

def gpu_voxel_downsample(pcd_t, voxel_size):
    try:
        return pcd_t.voxel_down_sample(voxel_size)
    except Exception as exc:
        if "out of memory" in str(exc).lower():
            print("GPU voxel downsample out of memory; retrying on CPU.", file=sys.stderr)
            cpu_dev = o3d.core.Device("CPU:0")
            pcd_cpu = pcd_t.to(cpu_dev)
            try:
                return pcd_cpu.voxel_down_sample(voxel_size)
            except Exception as cpu_exc:
                raise RuntimeError("Voxel downsample failed on CPU after GPU OOM.") from cpu_exc
        # fallback construction
        tmp = o3d.t.geometry.PointCloud()
        tmp.point["positions"] = pcd_t.point.positions
        if "colors" in pcd_t.point:
            tmp.point["colors"] = pcd_t.point["colors"]
        return tmp.voxel_down_sample(voxel_size)

def save_pointcloud_ply_legacy(pcd_t_ds, out_ply):
    pcd_cpu = o3d.geometry.PointCloud()
    pts = pcd_t_ds.point.positions.cpu().numpy()
    pcd_cpu.points = o3d.utility.Vector3dVector(pts)
    if "colors" in pcd_t_ds.point:
        try:
            cols = pcd_t_ds.point["colors"].cpu().numpy()
            cols = np.asarray(cols, dtype=np.float32)
            if cols.size and (cols.max() > 1.0 or cols.min() < 0.0):
                cols = np.clip(cols, 0.0, 255.0) / 255.0
            pcd_cpu.colors = o3d.utility.Vector3dVector(cols.astype(np.float64))
        except Exception:
            pass
    ok = o3d.io.write_point_cloud(out_ply, pcd_cpu, write_ascii=False)
    if not ok:
        raise RuntimeError(f"Failed to write PLY to {out_ply}")

def create_glb_from_points(positions: np.ndarray, colors: np.ndarray | None, out_glb: str):
    """
    Build a minimal GLB (glTF 2.0) storing points as a mesh primitive with mode=POINTS.
    Positions: (N,3) float32
    Colors: (N,3) float32 or None
    """
    # Ensure correct dtype
    positions = positions.astype(np.float32)
    n_points = positions.shape[0]
    has_color = colors is not None and colors.shape[0] == n_points
    if has_color:
        colors = colors.astype(np.float32)

    # Binary buffer: positions followed by colors (if any)
    pos_bytes = positions.tobytes()
    color_bytes = colors.tobytes() if has_color else b""
    # Align to 4-byte boundaries per glTF requirement
    def pad_to_4(b: bytes):
        rem = (4 - (len(b) % 4)) % 4
        return b + (b"\x00" * rem)
    pos_bytes_p = pad_to_4(pos_bytes)
    color_bytes_p = pad_to_4(color_bytes)
    bin_blob = pos_bytes_p + color_bytes_p
    total_length = len(bin_blob)

    gltf = GLTF2()
    gltf.asset = Asset(version="2.0")

    # Single buffer
    buffer = Buffer(byteLength=total_length)
    gltf.buffers = [buffer]

    # BufferViews
    bv_pos = BufferView(buffer=0, byteOffset=0, byteLength=len(pos_bytes_p), target=ARRAY_BUFFER)
    buffer_views = [bv_pos]
    if has_color:
        bv_color = BufferView(buffer=0, byteOffset=len(pos_bytes_p), byteLength=len(color_bytes_p), target=ARRAY_BUFFER)
        buffer_views.append(bv_color)
    gltf.bufferViews = buffer_views

    # Accessors
    # Positions accessor
    min_pos = positions.min(axis=0).tolist()
    max_pos = positions.max(axis=0).tolist()
    acc_pos = Accessor(bufferView=0, byteOffset=0, componentType=FLOAT, count=n_points, type=VEC3, min=min_pos, max=max_pos)
    accessors = [acc_pos]
    if has_color:
        # Colors accessor uses next bufferView
        acc_color = Accessor(bufferView=1, byteOffset=0, componentType=FLOAT, count=n_points, type=VEC3)
        accessors.append(acc_color)
    gltf.accessors = accessors

    # Mesh primitive
    prim = Primitive(attributes={"POSITION": 0})
    if has_color:
        prim.attributes["COLOR_0"] = 1
    prim.mode = POINTS  # 0: POINTS per pygltflib constants
    mesh = Mesh(primitives=[prim], name="pointcloud_points")
    gltf.meshes = [mesh]

    # Node + Scene
    node = Node(mesh=0, name="points_node")
    gltf.nodes = [node]
    scene = Scene(nodes=[0])
    gltf.scenes = [scene]
    gltf.scene = 0

    # Attach binary blob and write GLB
    gltf.set_binary_blob(bin_blob)
    gltf.save_binary(out_glb)

def main():
    args = parse_args()

    if not ensure_cuda():
        print("CUDA is not available in this Open3D installation. Exiting.")
        sys.exit(1)

    device = o3d.core.Device("CUDA:0")

    # Generate output file names from input file name
    input_path = Path(args.input_file_name)
    out_ply = str(input_path.with_stem(f"{input_path.stem}_downsampled").with_suffix(".ply"))
    out_glb = str(input_path.with_stem(f"{input_path.stem}_points").with_suffix(".glb"))

    print(f"Loading point cloud to device {device} ...")
    pcd_t = read_pointcloud_tensor(args.input_file_name, device)
    orig_n = len(pcd_t.point.positions)
    print(f"Original point count: {orig_n}")

    print(f"Downsampling (voxel_size={args.voxel}) on GPU ...")
    pcd_t_ds = gpu_voxel_downsample(pcd_t, args.voxel)
    ds_n = len(pcd_t_ds.point.positions)
    print(f"Downsampled point count: {ds_n}")

    # Save PLY (legacy)
    print(f"Saving downsampled PLY to {out_ply} ...")
    save_pointcloud_ply_legacy(pcd_t_ds, out_ply)

    # Prepare numpy arrays for GLB
    positions = pcd_t_ds.point.positions.cpu().numpy()
    colors = None
    if "colors" in pcd_t_ds.point:
        try:
            colors = pcd_t_ds.point["colors"].cpu().numpy()
            # Ensure colors are in 0..1; if in 0..255 normalize
            if colors.max() > 1.0:
                colors = colors / 255.0
        except Exception:
            colors = None

    print(f"Saving GLB (points) to {out_glb} ...")
    create_glb_from_points(positions, colors, out_glb)

    print("Done.")

if __name__ == "__main__":
    main()

