#!/usr/bin/env python3
# coding: utf-8

import os
import glob
import argparse
import numpy as np
import open3d as o3d
import time
import sys


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


class PLYSequenceViewer:
    def __init__(self, input_dir, loop=False, cumulative=False):
        """
        Initialize the PLY sequence viewer

        Args:
            input_dir (str): Directory containing PLY files
            loop (bool): Whether to loop the sequence or not
            cumulative (bool): Whether to accumulate point clouds or not
        """
        self.input_dir = input_dir
        self.loop = loop
        self.cumulative = cumulative
        self.quit_requested = False
        self.ply_files = []
        self.vis = None
        self.current_cloud = None
        self.is_first_cloud = True

        # For cumulative mode
        self.accumulated_points = []
        self.accumulated_colors = []

    def find_ply_files(self):
        """Find all PLY files in the input directory and sort them"""
        self.ply_files = sorted(glob.glob(os.path.join(self.input_dir, "*.ply")))
        if not self.ply_files:
            print(f"Error: No PLY files found in {self.input_dir}")
            sys.exit(1)
        print(f"Found {len(self.ply_files)} PLY files in {self.input_dir}")

    def initialize_visualizer(self):
        """Initialize the Open3D visualizer and create an empty point cloud"""
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(window_name="PLY Sequence Viewer", width=1280, height=720)
        # Register Q/q key callback for quitting
        self.vis.register_key_callback(ord("Q"), self.quit_callback)
        self.vis.register_key_callback(ord("q"), self.quit_callback)

        # Set rendering options
        render_option = self.vis.get_render_option()
        render_option.point_size = 1.0
        render_option.background_color = np.asarray([0.3, 0.3, 0.3])  # Dark grey background
        render_option.show_coordinate_frame = True  # Show coordinate frame for reference

        # Create an empty point cloud and add it to the visualizer
        self.current_cloud = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.current_cloud)

    def quit_callback(self, vis):
        """Callback when q/Q is pressed"""
        self.quit_requested = True
        return False

    def load_ply_with_intensity(self, filename):
        """
        Load a PLY file using plyfile and extract points and intensity (or reflectivity).

        Args:
            filename (str): Path to the PLY file

        Returns:
            tuple: (points, colors) or None if loading failed
        """
        try:
            from plyfile import PlyData

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

            # Convert intensity to jet colormap
            colors = intensity_to_jet_colormap(intensity)

            # Debug info
            point_count = points.shape[0]
            print(f"Loaded {point_count} points from {os.path.basename(filename)}")
            if point_count > 0:
                print(
                    f"Point cloud bounds: X:[{np.min(points[:,0]):.2f}, {np.max(points[:,0]):.2f}], "
                    f"Y:[{np.min(points[:,1]):.2f}, {np.max(points[:,1]):.2f}], "
                    f"Z:[{np.min(points[:,2]):.2f}, {np.max(points[:,2]):.2f}]"
                )

            return points, colors

        except Exception as e:
            print(f"Error loading PLY file {filename}: {e}")
            return None

    def update_visualization(self, points, colors):
        """
        Update the visualization with new point cloud data

        Args:
            points (numpy.ndarray): Point coordinates
            colors (numpy.ndarray): Point colors
        """
        if self.cumulative:
            # In cumulative mode, we accumulate points from all frames
            if len(self.accumulated_points) == 0:
                self.accumulated_points = points
                self.accumulated_colors = colors
            else:
                self.accumulated_points = np.vstack((self.accumulated_points, points))
                self.accumulated_colors = np.vstack((self.accumulated_colors, colors))

            # Use accumulated points for visualization
            self.current_cloud.points = o3d.utility.Vector3dVector(self.accumulated_points)
            self.current_cloud.colors = o3d.utility.Vector3dVector(self.accumulated_colors)

            # Print accumulation status
            print(f"Total accumulated points: {len(self.accumulated_points)}", end="\r")
        else:
            # In standard mode, we just update with the current frame
            self.current_cloud.points = o3d.utility.Vector3dVector(points)
            self.current_cloud.colors = o3d.utility.Vector3dVector(colors)

        # Reset view to see the whole point cloud if this is the first one
        if self.is_first_cloud:
            # Reset the view
            self.vis.reset_view_point(True)
            self.is_first_cloud = False

        # Update geometry and redraw
        self.vis.update_geometry(self.current_cloud)
        self.vis.poll_events()
        self.vis.update_renderer()

    def run_visualization(self):
        """Run the visualization loop"""
        try:
            self.find_ply_files()
            self.initialize_visualizer()

            # Process the first file separately to set up the view correctly
            if self.ply_files:
                first_file = self.ply_files[0]
                result = self.load_ply_with_intensity(first_file)
                if result is not None:
                    points, colors = result
                    print(f"Displaying initial frame: {os.path.basename(first_file)}")
                    self.update_visualization(points, colors)
                    # Give time for the window to initialize
                    time.sleep(1.0)
                else:
                    print(f"Failed to load initial frame: {first_file}")

            # Main loop for all files
            while not self.quit_requested:
                for i, ply_file in enumerate(self.ply_files):
                    if i == 0 and not self.cumulative:
                        # Skip the first file if not in cumulative mode (already processed)
                        continue

                    if self.quit_requested:
                        break

                    result = self.load_ply_with_intensity(ply_file)
                    if result is not None:
                        points, colors = result
                        if not self.cumulative:
                            print(f"Displaying: {os.path.basename(ply_file)}", end="\r")
                        self.update_visualization(points, colors)
                        time.sleep(0.05)
                    else:
                        print(f"Failed to load: {ply_file}")

                if not self.loop:
                    break

                # In cumulative mode with loop, reset accumulation at the end of each loop
                if self.cumulative and self.loop:
                    self.accumulated_points = []
                    self.accumulated_colors = []

            print("\nExiting visualization...")
            self.vis.destroy_window()

        except Exception as e:
            print(f"Error during visualization: {e}")
            if hasattr(self, "vis") and self.vis is not None:
                self.vis.destroy_window()


def main():
    parser = argparse.ArgumentParser(description="Visualize PLY file sequence with intensity using Open3D")
    parser.add_argument("-i", "--input-ply-dir", required=True, help="Directory containing PLY files")
    parser.add_argument("-l", "--loop", action="store_true", help="Loop playback of point cloud sequence")
    parser.add_argument("-c", "--cumulative", action="store_true", help="Accumulate point clouds over time")
    args = parser.parse_args()

    if not os.path.isdir(args.input_ply_dir):
        print(f"Error: Directory '{args.input_ply_dir}' does not exist")
        return 1

    viewer = PLYSequenceViewer(args.input_ply_dir, args.loop, args.cumulative)
    viewer.run_visualization()
    return 0


if __name__ == "__main__":
    sys.exit(main())
