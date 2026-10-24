#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import sys
import time
from typing import Any, Dict, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
import yaml
from std_msgs.msg import Float64

from can_driver.srv import MotorCommand


def parse_motor_id(raw: str) -> int:
    return int(raw, 0)


def parse_joint_motor_id(raw: Any) -> Optional[int]:
    if isinstance(raw, int):
        return raw
    if isinstance(raw, str):
        try:
            return int(raw, 0)
        except ValueError:
            return None
    return None


def normalize_protocol(value: Any) -> str:
    if not isinstance(value, str):
        return ""
    return value.strip().upper()


def default_profile_path() -> str:
    pkg = get_package_share_directory("can_driver")
    return os.path.join(pkg, "config", "mt_control_profiles.yaml")


def load_profiles(path: str) -> Dict[str, Dict[str, Any]]:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    profiles = data.get("profiles", {})
    if not isinstance(profiles, dict):
        raise RuntimeError("profiles 节点不存在或格式错误")
    return profiles


class MtMotorInterface:
    def __init__(self, node, profile_name: str, profile: Dict[str, Any], driver_ns_override: str = ""):
        self.node = node
        self.profile_name = profile_name
        self.profile = profile
        ns = driver_ns_override.strip() or str(profile.get("driver_ns", "/can_driver_node"))
        self.driver_ns = ns.rstrip("/") if ns != "/" else ""

        self.motor_cmd_srv = f"{self.driver_ns}/motor_command"
        self.joints_param = f"{self.driver_ns}/joints"

    def _load_joints(self):
        config_file = self.node.get_parameter("config_file").value
        if not config_file:
            raise RuntimeError("未提供 config_file 参数；ROS2 节点不再使用 ROS1 参数服务器")
        with open(str(config_file), "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        node_cfg = data.get("can_driver_node", {})
        joints = node_cfg.get("joints", []) if isinstance(node_cfg, dict) else []
        if not isinstance(joints, list):
            raise RuntimeError(f"配置文件 joints 格式错误: {config_file}")
        return joints

    def resolve_joint_name(self, motor_id: int, joint_name_override: str = "") -> str:
        if joint_name_override:
            return joint_name_override

        can_device_expect = str(self.profile.get("can_device", ""))
        joints = self._load_joints()
        for j in joints:
            if not isinstance(j, dict):
                continue
            if normalize_protocol(j.get("protocol", "")) != "MT":
                continue

            jid = parse_joint_motor_id(j.get("motor_id"))
            if jid != motor_id:
                continue

            if can_device_expect and str(j.get("can_device", "")) != can_device_expect:
                continue

            name = j.get("name", "")
            if isinstance(name, str) and name:
                return name

        raise RuntimeError(
            f"无法在 {self.joints_param} 中找到 MT 关节: motor_id={hex(motor_id)} "
            f"(profile={self.profile_name}, can_device={can_device_expect})"
        )

    def call_motor_command(self, motor_id: int, command: int, value: float = 0.0) -> None:
        client = self.node.create_client(MotorCommand, self.motor_cmd_srv)
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"服务不可用: {self.motor_cmd_srv}")
        req = MotorCommand.Request()
        req.motor_id = motor_id
        req.command = command
        req.value = value
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.exception() is not None:
            raise RuntimeError(str(future.exception()))
        resp = future.result()
        if not resp.success:
            raise RuntimeError(f"service 失败: {resp.message}")

    def publish_direct(self, joint_name: str, is_velocity: bool, value: float) -> None:
        suffix = "cmd_velocity" if is_velocity else "cmd_position"
        topic = f"{self.driver_ns}/motor/{joint_name}/{suffix}"

        pub = self.node.create_publisher(Float64, topic, 1)
        start = time.time()
        while pub.get_subscription_count() == 0 and (time.time() - start) < 1.0 and rclpy.ok():
            time.sleep(0.05)

        msg = Float64()
        msg.data = value
        for _ in range(3):
            pub.publish(msg)
            time.sleep(0.03)

    def check_motor_in_profile(self, motor_id: int) -> None:
        ids = self.profile.get("mt_motor_ids", [])
        if not isinstance(ids, list) or not ids:
            return
        normalized = set()
        for x in ids:
            try:
                normalized.add(int(x, 0) if isinstance(x, str) else int(x))
            except (ValueError, TypeError):
                continue
        if normalized and motor_id not in normalized:
            raise RuntimeError(
                f"motor_id={hex(motor_id)} 不在 profile '{self.profile_name}' 允许列表中: "
                f"{[hex(i) for i in sorted(normalized)]}"
            )


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="MT 电机控制接口（含 MIT 位置模式）")
    p.add_argument("--profile", required=True, help="车配置名，如 car_a/car_b")
    p.add_argument(
        "--action",
        required=True,
        choices=["enable", "disable", "stop", "mode", "velocity", "position", "list"],
        help="控制动作",
    )
    p.add_argument("--motor-id", default="0x141", help="目标电机ID，支持十进制/十六进制")
    p.add_argument("--value", type=float, default=0.0, help="动作参数：mode用0/1，速度/位置用浮点")
    p.add_argument("--joint", default="", help="可选：直接指定 joint 名，跳过自动匹配")
    p.add_argument("--profiles-file", default=default_profile_path(), help="profile 配置文件路径")
    p.add_argument("--config-file", default="", help="can_driver ROS2 YAML 配置文件路径")
    p.add_argument("--driver-ns", default="", help="可选：覆盖 profile 内 driver_ns")
    p.add_argument(
        "--no-auto-mode",
        action="store_true",
        help="禁用 velocity/position 动作中的自动模式切换（默认会自动切换）",
    )
    return p


def print_profiles(profiles: Dict[str, Dict[str, Any]]) -> None:
    print("可用 MT profiles:")
    for name, cfg in profiles.items():
        desc = cfg.get("description", "")
        dev = cfg.get("can_device", "")
        ids = cfg.get("mt_motor_ids", [])
        print(f"- {name}: {desc}")
        print(f"    can_device: {dev}")
        print(f"    mt_motor_ids: {ids}")


def main() -> int:
    args = build_parser().parse_args()

    try:
        profiles = load_profiles(args.profiles_file)
    except Exception as e:
        print(f"[MT-IF] 加载 profiles 失败: {e}", file=sys.stderr)
        return 2

    if args.action == "list":
        print_profiles(profiles)
        return 0

    if args.profile not in profiles:
        print(f"[MT-IF] 未找到 profile: {args.profile}", file=sys.stderr)
        print_profiles(profiles)
        return 2

    try:
        motor_id = parse_motor_id(args.motor_id)
    except ValueError:
        print(f"[MT-IF] motor-id 无法解析: {args.motor_id}", file=sys.stderr)
        return 2

    rclpy.init(args=None)
    node = rclpy.create_node("mt_motor_interface_cli")
    node.declare_parameter("config_file", "")
    if args.config_file:
        node.set_parameters([rclpy.parameter.Parameter("config_file", value=args.config_file)])
    else:
        default_config = os.path.join(get_package_share_directory("can_driver"), "config", "can_driver.yaml")
        node.set_parameters([rclpy.parameter.Parameter("config_file", value=default_config)])
    iface = MtMotorInterface(node, args.profile, profiles[args.profile], args.driver_ns)

    try:
        iface.check_motor_in_profile(motor_id)

        if args.action == "enable":
            iface.call_motor_command(motor_id, MotorCommand.Request.CMD_ENABLE, 0.0)
            print("OK enable")
            return 0

        if args.action == "disable":
            iface.call_motor_command(motor_id, MotorCommand.Request.CMD_DISABLE, 0.0)
            print("OK disable")
            return 0

        if args.action == "stop":
            iface.call_motor_command(motor_id, MotorCommand.Request.CMD_STOP, 0.0)
            print("OK stop")
            return 0

        if args.action == "mode":
            if args.value not in (0.0, 1.0):
                raise RuntimeError("mode 动作要求 --value 只能是 0(position/MIT) 或 1(velocity)")
            iface.call_motor_command(motor_id, MotorCommand.Request.CMD_SET_MODE, args.value)
            print("OK mode")
            return 0

        if args.action in ("velocity", "position"):
            joint = iface.resolve_joint_name(motor_id, args.joint)
            if not args.no_auto_mode:
                # 明确设置模式：velocity=1，position(MIT)=0
                mode_value = 1.0 if args.action == "velocity" else 0.0
                iface.call_motor_command(motor_id, MotorCommand.Request.CMD_SET_MODE, mode_value)
            iface.publish_direct(joint, args.action == "velocity", args.value)
            print(f"OK {args.action} joint={joint} value={args.value}")
            return 0

        raise RuntimeError(f"不支持动作: {args.action}")
    except Exception as e:
        print(f"[MT-IF] 执行失败: {e}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
