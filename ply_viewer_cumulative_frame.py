#!/usr/bin/env python3
# coding: utf-8

import argparse
import os
import glob
import numpy as np
import open3d as o3d
from plyfile import PlyData


def intensity_to_jet_colormap(intensity):
    """
    Convert intensity values (0-1) to jet colormap (blue->cyan->green->yellow->red)

    Args:
        intensity (numpy.ndarray): Intensity values between 0 and 1

    Returns:
        numpy.ndarray: RGB colors with shape (N, 3)
    """
    # Create empty RGB array
    colors = np.zeros((intensity.shape[0], 3))

    # Blue to cyan (0.0 - 0.25)
    idx = (intensity >= 0.0) & (intensity < 0.25)
    colors[idx, 0] = 0  # R
    colors[idx, 1] = 4 * intensity[idx]  # G (0 -> 1)
    colors[idx, 2] = 1  # B

    # Cyan to green (0.25 - 0.5)
    idx = (intensity >= 0.25) & (intensity < 0.5)
    colors[idx, 0] = 0  # R
    colors[idx, 1] = 1  # G
    colors[idx, 2] = 1 + 4 * (0.25 - intensity[idx])  # B (1 -> 0)

    # Green to yellow (0.5 - 0.75)
    idx = (intensity >= 0.5) & (intensity < 0.75)
    colors[idx, 0] = 4 * (intensity[idx] - 0.5)  # R (0 -> 1)
    colors[idx, 1] = 1  # G
    colors[idx, 2] = 0  # B

    # Yellow to red (0.75 - 1.0)
    idx = (intensity >= 0.75) & (intensity <= 1.0)
    colors[idx, 0] = 1  # R
    colors[idx, 1] = 1 + 4 * (0.75 - intensity[idx])  # G (1 -> 0)
    colors[idx, 2] = 0  # B

    return colors


def load_ply_with_intensity(filename, use_jet_colormap=False):
    """
    Load a PLY file using plyfile and extract points and intensity (or reflectivity).

    Args:
        filename (str): Path to the input PLY file
        use_jet_colormap (bool): Whether to use jet colormap instead of grayscale

    Returns:
        tuple: (points, colors) or (None, None) on failure
    """
    try:
        ply_data = PlyData.read(filename)
        vertex_data = ply_data["vertex"].data

        # Extract x, y, z coordinates
        points = np.vstack((vertex_data["x"], vertex_data["y"], vertex_data["z"])).T

        # Check for intensity or reflectivity field
        if "reflectivity" in vertex_data.dtype.names:
            intensity = np.array(vertex_data["reflectivity"], dtype=np.float32)
        elif "intensity" in vertex_data.dtype.names:
            intensity = np.array(vertex_data["intensity"], dtype=np.float32)
        else:
            # Use default value if neither field exists
            intensity = np.ones(points.shape[0], dtype=np.float32) * 0.7

        # Normalize intensity (scale to 0-1 range if max > 1.0)
        max_val = intensity.max()
        if max_val > 1.0:
            intensity = intensity / (255.0 if max_val > 255 else max_val)

        # Convert intensity to colors
        if use_jet_colormap:
            colors = intensity_to_jet_colormap(intensity)
        else:
            # Create grayscale color array
            colors = np.tile(intensity.reshape(-1, 1), (1, 3))

        return points, colors

    except Exception as e:
        print(f"Error loading PLY file '{filename}': {e}")
        return None, None


def find_ply_files(directory):
    """
    Find all PLY files in the given directory and sort them.

    Args:
        directory (str): Directory to search for PLY files

    Returns:
        list: Sorted list of PLY file paths
    """
    ply_files = sorted(glob.glob(os.path.join(directory, "*.ply")))
    return ply_files


def extract_frame_number(filename):
    """
    Extract frame number from filename, assuming format like 'frame_000123.ply'.

    Args:
        filename (str): Filename to extract frame number from

    Returns:
        int: Frame number or None if not found
    """
    try:
        basename = os.path.basename(filename)
        # Remove extension
        basename = os.path.splitext(basename)[0]
        # Split by underscore and get the last part
        parts = basename.split("_")
        if len(parts) > 1:
            return int(parts[-1])
        return None
    except (ValueError, IndexError):
        return None


def main():
    parser = argparse.ArgumentParser(description="Accumulated PLY Sequence Viewer")
    parser.add_argument("-i", "--input-ply-dir", required=True, help="Directory containing PLY files")
    parser.add_argument("--start", type=int, default=0, help="Start frame number (inclusive)")
    parser.add_argument("--end", type=int, default=-1, help="End frame number (inclusive, -1 for all)")
    parser.add_argument("--jet-colormap", action="store_true", help="Use jet colormap for visualization")
    args = parser.parse_args()

    if not os.path.isdir(args.input_ply_dir):
        print(f"Error: Directory '{args.input_ply_dir}' does not exist")
        return 1

    # Find all PLY files
    ply_files = find_ply_files(args.input_ply_dir)

    if not ply_files:
        print(f"Error: No PLY files found in {args.input_ply_dir}")
        return 1

    print(f"Found {len(ply_files)} PLY files in {args.input_ply_dir}")

    # If end frame is not specified, use the maximum frame number
    max_frame = len(ply_files) - 1
    if args.end < 0:
        args.end = max_frame

    # Validate frame range
    if args.start < 0 or args.start > max_frame:
        print(f"Error: Start frame {args.start} is out of range (0-{max_frame})")
        return 1

    if args.end < args.start or args.end > max_frame:
        print(f"Error: End frame {args.end} is out of range ({args.start}-{max_frame})")
        return 1

    # Select files within the specified range
    selected_files = ply_files[args.start : args.end + 1]
    print(f"Selected {len(selected_files)} files from frames {args.start} to {args.end}")

    # Accumulated points and colors
    accumulated_points = []
    accumulated_colors = []

    # Load and accumulate point cloud data
    for i, ply_file in enumerate(selected_files):
        print(f"Loading file {i+1}/{len(selected_files)}: {os.path.basename(ply_file)}")

        points, colors = load_ply_with_intensity(ply_file, args.jet_colormap)
        if points is not None and colors is not None:
            if len(accumulated_points) == 0:
                accumulated_points = points
                accumulated_colors = colors
            else:
                accumulated_points = np.vstack((accumulated_points, points))
                accumulated_colors = np.vstack((accumulated_colors, colors))

    if len(accumulated_points) == 0:
        print("Error: No valid point data found in the selected frames")
        return 1

    print(f"Total accumulated points: {len(accumulated_points)}")

    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(accumulated_points)
    pcd.colors = o3d.utility.Vector3dVector(accumulated_colors)

    # Display the accumulated point cloud
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Accumulated PLY Viewer", width=1280, height=720)
    vis.add_geometry(pcd)

    # Set rendering options
    render_option = vis.get_render_option()
    render_option.point_size = 1.0
    render_option.background_color = np.asarray([0.3, 0.3, 0.3])  # Dark grey background
    render_option.show_coordinate_frame = True

    # Run visualization (blocks until window is closed)
    vis.run()
    vis.destroy_window()

    return 0


if __name__ == "__main__":
    import sys

    sys.exit(main())
