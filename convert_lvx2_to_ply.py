#!/usr/bin/env python3
# coding: utf-8

import struct
import os
import argparse
import numpy as np
from dataclasses import dataclass, field
from datetime import datetime
from plyfile import PlyData, PlyElement


@dataclass
class PublicHeader:
    signature: str = ""
    ver_a: int = 0
    ver_b: int = 0
    ver_c: int = 0
    ver_d: int = 0
    magic_code: int = 0


@dataclass
class PrivateHeader:
    duration: int = 0
    device_count: int = 0


@dataclass
class DeviceInformation:
    lidar_sn: str = ""
    hub_sn: str = ""
    lidar_id: int = 0
    lidar_type: int = 0
    device_type: int = 0
    enable_extrinsic: int = 0
    offset_roll: float = 0.0
    offset_pitch: float = 0.0
    offset_yaw: float = 0.0
    offset_x: float = 0.0
    offset_y: float = 0.0
    offset_z: float = 0.0


@dataclass
class FrameHeader:
    current_offset: int = 0
    next_offset: int = 0
    frame_index: int = 0
    frame_size: int = 0


@dataclass
class PackageHeader:
    version: int = 0
    lidar_id: int = 0
    lidar_type: int = 0
    timestamp_type: int = 0
    timestamp: float = 0.0
    udp_count: int = 0
    data_type: int = 0
    length: int = 0
    frame_count: int = 0
    points_count: int = 0


@dataclass
class Point:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    reflectivity: int = 0
    tag: int = 0


@dataclass
class Package:
    header: PackageHeader = PackageHeader()
    points: list = field(default_factory=list)


@dataclass
class Frame:
    header: FrameHeader = FrameHeader()
    timestamp: float = 0.0
    packages: list = field(default_factory=list)


# Pop n items from list object (FIFO)
def pop_n(size: int, data: list):
    return (data[:size], data[size:])


# LVX2 file parser class
class LVX2_PARSER(object):
    # Parse public header
    def parse_public_header(self, buf: bytes):
        if len(buf) != 24:
            print("Invalid data length for public header")
            return None

        ret = PublicHeader()
        ret.signature = buf[0:16].decode(encoding="utf-8", errors="ignore")
        ret.ver_a = int(struct.unpack("<b", buf[16:17])[0])
        ret.ver_b = int(struct.unpack("<b", buf[17:18])[0])
        ret.ver_c = int(struct.unpack("<b", buf[18:19])[0])
        ret.ver_d = int(struct.unpack("<b", buf[19:20])[0])
        ret.magic_code = int(struct.unpack("<L", buf[20:24])[0])

        # Signature check
        if not ret.signature.startswith("livox_tech"):
            print("Signature Error (livox_tech):", ret.signature)
            return None
        else:
            print("Signature OK:", ret.signature)

        # Magic check
        if ret.magic_code != 0xAC0EA767:
            print("Magic Code Error (0xAC0EA767):", ret.magic_code)
            return None
        else:
            print("Magic Code OK:", hex(ret.magic_code))

        return ret

    # Parse private header
    def parse_private_header(self, buf: bytes):
        if len(buf) != 5:
            print("Invalid data length for private header")
            return None

        ret = PrivateHeader()
        ret.duration = float(struct.unpack("<L", buf[0:4])[0]) / 1.0e3  # [sec] duration time
        ret.device_count = int(struct.unpack("<b", buf[4:5])[0])  # device counts
        print("Data duration:", ret.duration)
        print("Devices:", ret.device_count)

        return ret

    # Parse device information
    def parse_device_information(self, buf: bytes):
        if len(buf) != 63:
            print("Invalid data length for device information")
            return None

        ret = DeviceInformation()
        ret.lidar_sn = buf[0:16].decode(encoding="utf-8", errors="ignore")  # LiDAR S/N
        ret.hub_sn = buf[16:32].decode(encoding="utf-8", errors="ignore")  # device S/N
        ret.lidar_id = int(struct.unpack("<L", buf[32:36])[0])  # LiDAR ID
        ret.lidar_type = int(struct.unpack("<b", buf[36:37])[0])  # LiDAR type
        ret.device_type = int(struct.unpack("<b", buf[37:38])[0])  # device type
        ret.enable_extrinsic = int(struct.unpack("<b", buf[38:39])[0])  # extrinsic enable
        ret.offset_roll = float(struct.unpack("<f", buf[39:43])[0])  # [deg] offset roll(X axis)
        ret.offset_pitch = float(struct.unpack("<f", buf[43:47])[0])  # [deg] offset pitch(Y axis)
        ret.offset_yaw = float(struct.unpack("<f", buf[47:51])[0])  # [deg] offset yaw(Z axis)
        ret.offset_x = float(struct.unpack("<f", buf[51:55])[0])  # [m] offset X
        ret.offset_y = float(struct.unpack("<f", buf[55:59])[0])  # [m] offset Y
        ret.offset_z = float(struct.unpack("<f", buf[59:63])[0])  # [m] offset Z

        print("LiDAR SN   :", ret.lidar_sn)
        print("HUB SN      :", ret.hub_sn)
        print("LiDAR ID    :", ret.lidar_id)
        if ret.device_type == 9:
            print("Device Type : MID-360")
        elif ret.device_type == 10:
            print("Device Type : HAP")
        else:
            print("Device Type : Unknown")

        if ret.enable_extrinsic:
            print("Extrinsic   : True")
        else:
            print("Extrinsic   : False")
        print("Offset rotation :", (ret.offset_roll, ret.offset_pitch, ret.offset_yaw))
        print("Offset position :", (ret.offset_x, ret.offset_y, ret.offset_z))

        return ret

    # Parse frame header
    def parse_frame_header(self, buf: bytes):
        if len(buf) != 24:
            print("Invalid data length for frame header")
            return None

        ret = FrameHeader()
        ret.current_offset = struct.unpack("<Q", buf[0:8])[0]
        ret.next_offset = struct.unpack("<Q", buf[8:16])[0]
        ret.frame_index = struct.unpack("<Q", buf[16:24])[0]
        ret.frame_size = ret.next_offset - ret.current_offset

        return ret

    # Parse package header
    def parse_package_header(self, buf: bytes):
        if len(buf) != 27:
            print("Invalid data length for package header")
            return None
        ret = PackageHeader()
        ret.version = int(struct.unpack("<b", buf[0:1])[0])
        ret.lidar_id = int(struct.unpack("<L", buf[1:5])[0])
        ret.lidar_type = int(struct.unpack("<b", buf[5:6])[0])
        ret.timestamp_type = int(struct.unpack("<b", buf[6:7])[0])
        ret.timestamp = float(struct.unpack("<Q", buf[7:15])[0]) / 1.0e9
        ret.udp_count = int(struct.unpack("<H", buf[15:17])[0])
        ret.data_type = int(struct.unpack("<b", buf[17:18])[0])
        ret.length = int(struct.unpack("<L", buf[18:22])[0])
        ret.frame_count = int(struct.unpack("<b", buf[22:23])[0])

        # Check data type
        if ret.data_type == 1:
            data_length = 14
        elif ret.data_type == 2:
            data_length = 8
        else:
            print("Invalid data type:", ret.data_type)
            return None

        # Check data size
        if ret.length % data_length != 0:
            print("Invalid point data size")
            return None

        ret.points_count = ret.length // data_length

        return ret

    # Parse points
    def parse_points(self, buf: bytes, data_type: int):
        if data_type == 1:
            data_length = 14
        elif data_type == 2:
            data_length = 8
        else:
            print("Invalid data type:", data_type)
            return None

        ret = []
        for _ in range(int(len(buf) / data_length)):
            p = Point()
            (_pbuf, buf) = pop_n(data_length, buf)
            if data_type == 1:
                p.x = struct.unpack("<l", _pbuf[0:4])[0] / 1.0e3
                p.y = struct.unpack("<l", _pbuf[4:8])[0] / 1.0e3
                p.z = struct.unpack("<l", _pbuf[8:12])[0] / 1.0e3
                p.reflectivity = struct.unpack("<b", _pbuf[12:13])[0]
                p.tag = struct.unpack("<b", _pbuf[13:14])[0]
            elif data_type == 2:
                p.x = struct.unpack("<h", _pbuf[0:2])[0] / 1.0e2
                p.y = struct.unpack("<h", _pbuf[2:4])[0] / 1.0e2
                p.z = struct.unpack("<h", _pbuf[4:6])[0] / 1.0e2
                p.reflectivity = struct.unpack("<b", _pbuf[6:7])[0]
                p.tag = struct.unpack("<b", _pbuf[7:8])[0]
            ret.append(p)

        return ret

    # Parse frame packages
    def parse_frame_packages(self, buf: bytes):
        ret = []

        while len(buf) > 0:
            # Read package header
            (_data, buf) = pop_n(27, buf)
            pkg = Package()
            pkg.header = self.parse_package_header(_data)

            if pkg.header is None:
                return None

            # Parse points
            (_data, buf) = pop_n(pkg.header.length, buf)
            pkg.points = self.parse_points(_data, pkg.header.data_type)
            if pkg.points is None:
                return None

            # Add package
            ret.append(pkg)

        return ret

    # Constructor
    def __init__(self, in_file: str):
        # Initialize parameters
        self._in_file = in_file

        # Headers
        self._pub_header = PublicHeader()
        self._prv_header = PrivateHeader()
        self._devices = []
        self._frames = []

        try:
            # Open binary file
            with open(self._in_file, "rb") as _lvx2:
                # Read public header
                print("==================== Public Header ====================")
                _data = _lvx2.read(24)
                self._pub_header = self.parse_public_header(_data)
                if self._pub_header is None:
                    return

                # Read private header
                print("==================== Private Header ====================")
                _data = _lvx2.read(5)
                self._prv_header = self.parse_private_header(_data)
                if self._prv_header is None:
                    return

                # Read device information
                print("==================== Device Information ====================")

                for i in range(self._prv_header.device_count):
                    print("Device #{}".format(i))
                    _data = _lvx2.read(63)
                    _dev = self.parse_device_information(_data)

                    if _dev is None:
                        return

                    # Add device
                    self._devices.append(_dev)

                print("==================== Point Cloud Data ====================")
                frame_count = 0
                while True:
                    # Read frame data
                    _data = _lvx2.read(24)
                    if _data == b"":
                        print(f"End of file. Total frames: {frame_count}")
                        break

                    # Parse frame header
                    _frm = Frame()
                    _frm.header = self.parse_frame_header(_data)

                    if _frm.header is None:
                        return

                    # Parse packages
                    _data = _lvx2.read(_frm.header.frame_size - 24)
                    _frm.packages = self.parse_frame_packages(_data)

                    if _frm.packages is None:
                        return

                    # Add timestamp (average of all package timestamps in the frame)
                    _ts = 0.0
                    for _pkg in _frm.packages:
                        _ts = _ts + _pkg.header.timestamp
                    _frm.timestamp = _ts / len(_frm.packages)

                    # Add frame
                    self._frames.append(_frm)
                    frame_count += 1

                    if frame_count % 10 == 0:
                        print(f"Parsed {frame_count} frames...")

        except FileNotFoundError:
            print(f"File not found: {self._in_file}")
        except Exception as e:
            print(f"Error parsing LVX2 file: {e}")

    # Destructor
    def __del__(self):
        pass


class LVX2_to_PLY(object):
    def __init__(
        self, in_file: str, out_dir: str, prefix: str = "frame", start_idx: int = 0, end_idx: int = -1, step: int = 1
    ):
        # Initialize parameters
        self._in_file = in_file
        self._out_dir = out_dir
        self._prefix = prefix
        self._start_idx = start_idx
        self._end_idx = end_idx
        self._step = step

        # Create output directory if it doesn't exist
        os.makedirs(self._out_dir, exist_ok=True)

        # Parse LVX2 file
        print(f"Parsing LVX2 file: {self._in_file}")
        self._lvx2 = LVX2_PARSER(in_file=self._in_file)

        # Check if we have frames
        if not hasattr(self._lvx2, "_frames") or len(self._lvx2._frames) == 0:
            print("No frames found in the LVX2 file")
            return

        # Adjust end index if needed
        if self._end_idx < 0 or self._end_idx >= len(self._lvx2._frames):
            self._end_idx = len(self._lvx2._frames) - 1

        # Validate indices
        if self._start_idx < 0 or self._start_idx > self._end_idx:
            print("Invalid start/end indices")
            return

        # Convert frames to PLY
        self._convert_frames_to_ply()

    def _convert_frames_to_ply(self):
        # Total frames to process
        total_frames = len(range(self._start_idx, self._end_idx + 1, self._step))
        print(f"Converting {total_frames} frames to PLY format...")

        for i, frame_idx in enumerate(range(self._start_idx, self._end_idx + 1, self._step)):
            frame = self._lvx2._frames[frame_idx]

            # Prepare points data
            vertex_data = []

            # Collect points from all packages in the frame
            for package in frame.packages:
                for point in package.points:
                    # Add point to vertex data
                    vertex_data.append((point.x, point.y, point.z, point.reflectivity, point.tag))

            # If no points, skip this frame
            if not vertex_data:
                print(f"Frame {frame_idx} has no points, skipping")
                continue

            # Convert to numpy structured array
            vertex = np.array(
                vertex_data, dtype=[("x", "f4"), ("y", "f4"), ("z", "f4"), ("reflectivity", "u1"), ("tag", "u1")]
            )

            # Create PLY element
            vertex_element = PlyElement.describe(vertex, "vertex")

            # Create PLY data
            ply_data = PlyData([vertex_element], text=False)

            # Save to file
            output_path = os.path.join(self._out_dir, f"{self._prefix}_{frame_idx:06d}.ply")
            ply_data.write(output_path)

            # Progress report
            if (i + 1) % 10 == 0 or i + 1 == total_frames:
                print(f"Converted {i+1}/{total_frames} frames to PLY ({(i+1)/total_frames*100:.1f}%)")

    def __del__(self):
        pass


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Convert LVX2 file to sequential PLY files",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
                Examples:
                # Convert all frames in LVX2 file to PLY files
                python lvx2_to_ply.py --in_file data.lvx2 --out_dir ./output_ply

                # Convert only frames 10-50 with custom prefix
                python lvx2_to_ply.py --in_file data.lvx2 --out_dir ./output_ply --prefix cloud --start 10 --end 50

                # Convert every 5th frame
                python lvx2_to_ply.py --in_file data.lvx2 --out_dir ./output_ply --step 5
            """,
    )
    parser.add_argument("-i", "--in_file", type=str, required=True, help="Input LVX2 file")
    parser.add_argument("-o", "--out_dir", type=str, default="output_ply", help="Output directory for PLY files")
    parser.add_argument("--prefix", type=str, help="Prefix for output filenames", default="frame")
    parser.add_argument("--start", type=int, help="Start frame index (inclusive)", default=0)
    parser.add_argument("--end", type=int, help="End frame index (inclusive, -1 for all)", default=-1)
    parser.add_argument("--step", type=int, help="Frame step (1 = every frame, 2 = every other, etc.)", default=1)
    args = parser.parse_args(args)

    # Convert LVX2 to PLY
    start_time = datetime.now()
    converter = LVX2_to_PLY(
        in_file=args.in_file,
        out_dir=args.out_dir,
        prefix=args.prefix,
        start_idx=args.start,
        end_idx=args.end,
        step=args.step,
    )
    end_time = datetime.now()

    print(f"Conversion completed in {(end_time - start_time).total_seconds():.2f} seconds")
    return 0


if __name__ == "__main__":
    main()
