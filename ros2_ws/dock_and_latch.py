#!/usr/bin/env python3
"""
dock_and_latch.py

Updated version:
- requests LOCAL_POSITION_NED and ATTITUDE streams explicitly
- waits longer for first position data
- has a fallback to GLOBAL_POSITION_INT for altitude sanity checks
- keeps streaming position setpoints during approach
- publishes a Gazebo DetachableJoint attach topic at the end

Typical usage:
python3 dock_and_latch.py \
  --drone1 tcp:127.0.0.1:5760 \
  --drone2 tcp:127.0.0.1:5770 \
  --takeoff-alt 1.5 \
  --prelatch-x 0.35 \
  --final-x 0.08 \
  --attach-topic /model/drone1/detachable_joint/attach
"""

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Optional

from pymavlink import mavutil


@dataclass
class LocalPose:
    x: float
    y: float
    z: float
    yaw: Optional[float] = None


class Vehicle:
    def __init__(self, conn_str: str, name: str):
        self.name = name
        self.master = mavutil.mavlink_connection(conn_str)
        print(f"[{self.name}] Waiting for heartbeat on {conn_str} ...")
        self.master.wait_heartbeat(timeout=30)
        print(f"[{self.name}] Connected")

    def request_message_interval(self, message_id: int, hz: float) -> None:
        interval_us = int(1_000_000 / hz)
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            interval_us,
            0, 0, 0, 0, 0,
        )

    def enable_streams(self) -> None:
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10)

    def mode_id(self, mode_name: str) -> int:
        mode_mapping = self.master.mode_mapping()
        if not mode_mapping or mode_name not in mode_mapping:
            raise RuntimeError(f"[{self.name}] Mode {mode_name!r} not supported")
        return mode_mapping[mode_name]

    def set_mode(self, mode_name: str) -> None:
        mode_id = self.mode_id(mode_name)
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        print(f"[{self.name}] Requested mode {mode_name}")

    def wait_mode(self, mode_name: str, timeout: float = 15.0) -> None:
        wanted = self.mode_id(mode_name)
        t0 = time.time()
        while time.time() - t0 < timeout:
            hb = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if hb and getattr(hb, "custom_mode", None) == wanted:
                print(f"[{self.name}] Mode is now {mode_name}")
                return
        raise TimeoutError(f"[{self.name}] Timed out waiting for mode {mode_name}")

    def arm(self) -> None:
        self.master.arducopter_arm()
        print(f"[{self.name}] Arm requested")

    def wait_armed(self) -> None:
        self.master.motors_armed_wait()
