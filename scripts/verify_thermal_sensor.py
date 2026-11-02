#!/usr/bin/env python3
"""
Diagnostic Node for Thermal Sensor Verification (Phase F5.2B)
Subscribes to the thermal image topic, validates message schema, measures
frame rates, computes pixel intensity statistics, and saves a sample frame.
"""

import sys
import os
import time
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False


class ThermalSensorVerifier(Node):
    def __init__(self, topic: str, target_frames: int = 10, timeout_sec: float = 15.0,
                 expected_width: int = 320, expected_height: int = 240,
                 expected_encoding: str = "mono16", save_path: str = "/tmp/thermal_frame_sample.png"):
        super().__init__('thermal_sensor_verifier')
        self.topic = topic
        self.target_frames = target_frames
        self.timeout_sec = timeout_sec
        self.expected_width = expected_width
        self.expected_height = expected_height
        self.expected_encoding = expected_encoding
        self.save_path = save_path

        self.frames_received = 0
        self.wall_timestamps = []
        self.sim_timestamps = []
        self.sample_frame = None
        self.frame_metadata = {}
        self.pixel_stats = []
        self.validation_errors = []

        if CV_AVAILABLE:
            self.bridge = CvBridge()
        else:
            self.bridge = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.subscription = self.create_subscription(
            Image,
            self.topic,
            self.image_callback,
            qos
        )
        self.get_logger().info(f"Subscribed to thermal topic: {self.topic}")
        self.get_logger().info(
            f"Expecting {self.target_frames} frames ({self.expected_width}x{self.expected_height}, "
            f"{self.expected_encoding}) within {self.timeout_sec}s..."
        )

    def image_callback(self, msg: Image):
        now_wall = time.time()
        now_sim = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)

        self.wall_timestamps.append(now_wall)
        self.sim_timestamps.append(now_sim)
        self.frames_received += 1

        # Schema and format validation
        if msg.width != self.expected_width:
            err = f"Width mismatch: expected {self.expected_width}, got {msg.width}"
            if err not in self.validation_errors:
                self.validation_errors.append(err)

        if msg.height != self.expected_height:
            err = f"Height mismatch: expected {self.expected_height}, got {msg.height}"
            if err not in self.validation_errors:
                self.validation_errors.append(err)

        if msg.encoding != self.expected_encoding:
            err = f"Encoding mismatch: expected {self.expected_encoding}, got {msg.encoding}"
            if err not in self.validation_errors:
                self.validation_errors.append(err)

        bytes_per_pixel = 2 if self.expected_encoding in ['mono16', '16UC1', 'L16'] else 1
        expected_step = msg.width * bytes_per_pixel
        if msg.step != expected_step:
            err = f"Step mismatch: expected {expected_step}, got {msg.step}"
            if err not in self.validation_errors:
                self.validation_errors.append(err)

        expected_data_len = msg.height * msg.step
        if len(msg.data) != expected_data_len:
            err = f"Data length mismatch: expected {expected_data_len} bytes, got {len(msg.data)}"
            if err not in self.validation_errors:
                self.validation_errors.append(err)

        if self.frames_received == 1:
            self.frame_metadata = {
                'width': msg.width,
                'height': msg.height,
                'encoding': msg.encoding,
                'step': msg.step,
                'is_bigendian': msg.is_bigendian,
                'data_len': len(msg.data)
            }
            self.get_logger().info(
                f"First frame: {msg.width}x{msg.height}, {msg.encoding}, step={msg.step}, "
                f"endian={'big' if msg.is_bigendian else 'little'}"
            )

        # Parse pixel data with strict endianness
        np_frame = None
        if bytes_per_pixel == 2:
            dtype = np.dtype('>u2') if msg.is_bigendian else np.dtype('<u2')
            try:
                np_frame = np.frombuffer(msg.data, dtype=dtype).reshape((msg.height, msg.width))
            except Exception as e:
                err = f"Buffer reshape failed: {e}"
                if err not in self.validation_errors:
                    self.validation_errors.append(err)
        else:
            try:
                np_frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
            except Exception as e:
                err = f"Buffer reshape failed: {e}"
                if err not in self.validation_errors:
                    self.validation_errors.append(err)

        if np_frame is not None:
            self.sample_frame = np_frame
            f_min = float(np.min(np_frame))
            f_max = float(np.max(np_frame))
            f_mean = float(np.mean(np_frame))
            f_std = float(np.std(np_frame))
            self.pixel_stats.append((f_min, f_max, f_mean, f_std))

            # Bounds sanity check (e.g. 200K - 400K -> 20000 - 40000 counts)
            if bytes_per_pixel == 2:
                if f_min < 10000 or f_max > 50000:
                    err = f"Pixel values outside physically plausible range [10000, 50000]: min={f_min}, max={f_max}"
                    if err not in self.validation_errors:
                        self.validation_errors.append(err)


def main():
    parser = argparse.ArgumentParser(description="Strict verification of ROS 2 thermal camera image output.")
    parser.add_argument("--topic", type=str, default="/drone_0/thermal/image_raw", help="Thermal image topic")
    parser.add_argument("--frames", type=int, default=10, help="Required number of frames to collect")
    parser.add_argument("--timeout", type=float, default=20.0, help="Timeout in seconds")
    parser.add_argument("--expected-width", type=int, default=320, help="Expected image width")
    parser.add_argument("--expected-height", type=int, default=240, help="Expected image height")
    parser.add_argument("--expected-encoding", type=str, default="mono16", help="Expected image encoding")
    parser.add_argument("--output", type=str, default="/tmp/thermal_frame_sample.png", help="Path to save frame image")
    args = parser.parse_args()

    rclpy.init()
    node = ThermalSensorVerifier(
        topic=args.topic,
        target_frames=args.frames,
        timeout_sec=args.timeout,
        expected_width=args.expected_width,
        expected_height=args.expected_height,
        expected_encoding=args.expected_encoding,
        save_path=args.output
    )

    start_time = time.time()
    try:
        while rclpy.ok() and node.frames_received < node.target_frames:
            elapsed = time.time() - start_time
            if elapsed > node.timeout_sec:
                node.get_logger().error(
                    f"Timeout ({node.timeout_sec}s) reached! Received {node.frames_received}/{node.target_frames} frames."
                )
                break
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    # Strict Acceptance Criteria Evaluation
    failure_reasons = []

    if node.frames_received < node.target_frames:
        failure_reasons.append(
            f"Insufficient frames received: got {node.frames_received}, required {node.target_frames}"
        )

    if node.validation_errors:
        failure_reasons.extend(node.validation_errors)

    if not node.pixel_stats:
        failure_reasons.append("No valid pixel statistics could be extracted from image stream")

    passed = (len(failure_reasons) == 0)

    # Analysis & Reporting
    print("\n" + "="*70)
    print("THERMAL SENSOR RIGOROUS VERIFICATION REPORT (PHASE F5.3A)")
    print("="*70)
    print(f"Topic:                 {node.topic}")
    print(f"Frames Target/Got:     {node.target_frames} / {node.frames_received}")

    meta = node.frame_metadata
    print(f"Resolution:            {meta.get('width', 'N/A')} x {meta.get('height', 'N/A')} (Expected: {args.expected_width}x{args.expected_height})")
    print(f"Encoding:              {meta.get('encoding', 'N/A')} (Expected: {args.expected_encoding})")
    print(f"Step / Data Length:    {meta.get('step', 'N/A')} bytes / {meta.get('data_len', 'N/A')} bytes")
    print(f"Endianness:            {'big' if meta.get('is_bigendian') else 'little'}")

    # Rate and RTF Analysis
    if len(node.wall_timestamps) > 1 and len(node.sim_timestamps) > 1:
        wall_dur = node.wall_timestamps[-1] - node.wall_timestamps[0]
        wall_rate = (len(node.wall_timestamps) - 1) / wall_dur if wall_dur > 0 else 0

        sim_dur = node.sim_timestamps[-1] - node.sim_timestamps[0]
        sim_rate = (len(node.sim_timestamps) - 1) / sim_dur if sim_dur > 0 else 0

        rtf = sim_dur / wall_dur if wall_dur > 0 else 0
        print("-" * 70)
        print("TEMPORAL ANALYSIS:")
        print(f"Wall-Clock Duration:   {wall_dur:.3f} s")
        print(f"Wall-Clock Frame Rate: {wall_rate:.2f} Hz")
        print(f"Simulation Duration:   {sim_dur:.3f} s")
        print(f"Simulation Frame Rate: {sim_rate:.2f} Hz (Target sensor rate: 15.00 Hz)")
        print(f"Effective Sim RTF:     {rtf:.3f}")

    if node.pixel_stats:
        mins = [s[0] for s in node.pixel_stats]
        maxs = [s[1] for s in node.pixel_stats]
        means = [s[2] for s in node.pixel_stats]
        stds = [s[3] for s in node.pixel_stats]
        print("-" * 70)
        print("PIXEL INTENSITY & TEMPERATURE CALIBRATION:")
        print(f"Raw Intensity Min:     {min(mins):.1f}")
        print(f"Raw Intensity Max:     {max(maxs):.1f}")
        print(f"Raw Intensity Mean:    {np.mean(means):.2f}")
        print(f"Raw Intensity StdDev:  {np.mean(stds):.2f}")

        # Linear resolution: 0.01 K/count for mono16
        if args.expected_encoding in ['mono16', '16UC1', 'L16']:
            t_min_k = min(mins) * 0.01
            t_max_k = max(maxs) * 0.01
            t_mean_k = np.mean(means) * 0.01
            print(f"Calibrated Kelvin:     Min={t_min_k:.2f} K, Max={t_max_k:.2f} K, Mean={t_mean_k:.2f} K")
            print(f"Calibrated Celsius:    Min={t_min_k - 273.15:.2f}°C, Max={t_max_k - 273.15:.2f}°C, Mean={t_mean_k - 273.15:.2f}°C")

    if node.sample_frame is not None and CV_AVAILABLE:
        try:
            os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
            sf = node.sample_frame
            if sf.dtype == np.uint16:
                save_img = cv2.normalize(sf, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            else:
                save_img = sf
            cv2.imwrite(args.output, save_img)
            print(f"Sample Frame Saved:    {args.output}")
        except Exception as e:
            print(f"Failed to save image:  {e}")

    print("=" * 70)
    if passed:
        print("[ACCEPTANCE RESULT] PASS — All mandatory sensor acceptance criteria met.")
        print("=" * 70)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    else:
        print("[ACCEPTANCE RESULT] FAIL — Rejection reasons:")
        for r in failure_reasons:
            print(f"  * {r}")
        print("=" * 70)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)


if __name__ == '__main__':
    main()
