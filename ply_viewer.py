#!/usr/bin/env python3
# coding: utf-8

import argparse
import os
import numpy as np
import open3d as o3d
from plyfile import PlyData


def load_ply_with_intensity(filename):
    """
    Load a PLY file using plyfile and extract points and intensity (or reflectivity).

    Args:
        filename (str): Path to the input PLY file

    Returns:
        tuple: (points, colors) or (None, None) on failure
    """
    try:
        ply_data = PlyData.read(filename)
        vertex_data = ply_data["vertex"].data

        # 抽出: x, y, z 座標 (N×3 の ndarray)
        points = np.vstack((vertex_data["x"], vertex_data["y"], vertex_data["z"])).T

        # intensity または reflectivity のフィールド確認
        if "reflectivity" in vertex_data.dtype.names:
            intensity = np.array(vertex_data["reflectivity"], dtype=np.float32)
        elif "intensity" in vertex_data.dtype.names:
            intensity = np.array(vertex_data["intensity"], dtype=np.float32)
        else:
            # フィールドが無い場合は、デフォルト値（0.7）を利用
            intensity = np.ones(points.shape[0], dtype=np.float32) * 0.7

        # intensityの正規化（最大値が1.0より大きければ0-1の範囲に変換）
        max_val = intensity.max()
        if max_val > 1.0:
            intensity = intensity / (255.0 if max_val > 255 else max_val)

        # 各点に対し、グレースケールの色 (R=G=B) を作成
        colors = np.tile(intensity.reshape(-1, 1), (1, 3))
        return points, colors

    except Exception as e:
        print(f"Error loading PLY file '{filename}': {e}")
        return None, None


def main():
    parser = argparse.ArgumentParser(description="Simple PLY Viewer with intensity using Open3D")
    parser.add_argument("-i", "--input-ply", required=True, help="Path to the input PLY file")
    args = parser.parse_args()

    if not os.path.isfile(args.input_ply):
        print(f"Error: File '{args.input_ply}' does not exist")
        return 1

    points, colors = load_ply_with_intensity(args.input_ply)
    if points is None or colors is None:
        print("Failed to load point cloud from the PLY file.")
        return 1

    # Open3DのPointCloudオブジェクトを生成し、点と色を設定
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # シンプルなウィンドウで表示 (ウィンドウが閉じられるまでブロッキング)
    o3d.visualization.draw_geometries([pcd], window_name="PLY Viewer", width=1280, height=720)

    return 0


if __name__ == "__main__":
    import sys

    sys.exit(main())
