#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
can_driver 多电机实时调试 UI（适配新版生命周期架构）

功能：
  - 生命周期管理：Init / Init+Release / Enable / Release/Resume / Halt / Shutdown / Recover
  - 多电机实时状态监控（位置/速度/电流/使能/故障）
  - 滑条实时发送位置或速度命令
  - 支持 position / velocity / csp 三种模式切换
  - 全局一键操作 + 单电机独立操作

用法：
  rosrun can_driver realtime_motor_ui.py
  rosrun can_driver realtime_motor_ui.py _init_loopback:=false _stream_hz:=100
"""

import threading
import tkinter as tk
from tkinter import ttk
import math
from pathlib import Path

import rospy
import rospkg
import yaml
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from can_driver.msg import MotorState
from can_driver.srv import Init, MotorCommand, Recover, SetZeroLimit, Shutdown

# MotorCommand.srv 常量
CMD_ENABLE = 0
CMD_DISABLE = 1
CMD_STOP = 2
CMD_SET_MODE = 3

# 模式值映射
MODE_MAP = {"position": 0.0, "velocity": 1.0, "csp": 2.0}
MODE_NAMES = {0: "未知", 1: "位置", 2: "速度", 3: "CSP", 5: "CSP"}

NS = "/can_driver_node"


def normalize_scale(value, default=1.0):
    """与 JointConfigParser 保持一致：>=2 视为 PPR，换算成 2pi/PPR。"""
    try:
        scale = float(value)
    except (TypeError, ValueError):
        return float(default)

    if not math.isfinite(scale) or scale <= 0.0:
        return float(default)
    if scale >= 2.0:
        return 2.0 * math.pi / scale
    return scale


class MotorPanel:
    """单个电机的状态显示 + 命令区域"""

    def __init__(self, parent: ttk.Frame, joint_cfg: dict, motor_cmd_proxy):
        self.name = joint_cfg["name"]
        self.motor_id = joint_cfg["motor_id"]
        self.default_mode = joint_cfg["control_mode"]
        self.position_scale = joint_cfg["position_scale"]
        self.velocity_scale = joint_cfg["velocity_scale"]
        self.motor_cmd = motor_cmd_proxy

        self.position = tk.StringVar(value="0.0000")
        self.velocity = tk.StringVar(value="0.0000")
        self.current = tk.StringVar(value="0")
        self.enabled = tk.BooleanVar(value=False)
        self.fault = tk.BooleanVar(value=False)
        self.mode_text = tk.StringVar(value="--")
        self.feedback_text = tk.StringVar(value="未知")
        self.last_mode_name = "unknown"

        self.frame = ttk.LabelFrame(parent, text=self.name, padding=4)

        # --- 状态行 ---
        row_status = ttk.Frame(self.frame)
        row_status.pack(fill=tk.X, pady=2)

        ttk.Label(row_status, text=f"ID=0x{self.motor_id:02X}", width=10).pack(
            side=tk.LEFT
        )
        ttk.Label(row_status, text="模式:").pack(side=tk.LEFT)
        ttk.Label(row_status, textvariable=self.mode_text, width=6).pack(side=tk.LEFT)

        self.lbl_enabled = ttk.Label(row_status, text="● 已使能", foreground="gray")
        self.lbl_enabled.pack(side=tk.LEFT, padx=8)
        self.lbl_fault = ttk.Label(row_status, text="● 故障", foreground="gray")
        self.lbl_fault.pack(side=tk.LEFT, padx=4)
        ttk.Label(row_status, text="反馈:").pack(side=tk.LEFT, padx=(6, 0))
        self.lbl_feedback = ttk.Label(row_status, textvariable=self.feedback_text, foreground="gray")
        self.lbl_feedback.pack(side=tk.LEFT, padx=4)

        ttk.Label(row_status, text="位置(rad):").pack(side=tk.LEFT, padx=(12, 0))
        ttk.Label(row_status, textvariable=self.position, width=10).pack(side=tk.LEFT)
        ttk.Label(row_status, text="速度(rad/s):").pack(side=tk.LEFT, padx=(6, 0))
        ttk.Label(row_status, textvariable=self.velocity, width=8).pack(side=tk.LEFT)
        ttk.Label(row_status, text="电流(raw):").pack(side=tk.LEFT, padx=(6, 0))
        ttk.Label(row_status, textvariable=self.current, width=8).pack(side=tk.LEFT)

    def update_state(self, msg: MotorState):
        if msg.position_valid:
            position_si = float(msg.position) * self.position_scale
            self.position.set(f"{position_si:.4f}")
        else:
            self.position.set("--")

        if msg.velocity_valid:
            velocity_si = float(msg.velocity) * self.velocity_scale
            self.velocity.set(f"{velocity_si:.4f}")
        else:
            self.velocity.set("--")

        self.current.set(str(msg.current) if msg.current_valid else "--")
        self.enabled.set(msg.enabled if msg.status_valid else False)
        self.fault.set(msg.fault if msg.status_valid else False)
        self.mode_text.set(MODE_NAMES.get(msg.mode, f"({msg.mode})") if msg.mode_valid else "未知")
        self.last_mode_name = (
            "position"
            if msg.mode == MotorState.MODE_POSITION
            else "velocity"
            if msg.mode == MotorState.MODE_VELOCITY
            else "csp"
            if msg.mode == MotorState.MODE_CSP
            else "unknown"
        )

        if msg.status_valid:
            if msg.enabled:
                self.lbl_enabled.config(text="● 已使能", foreground="green")
            else:
                self.lbl_enabled.config(text="● 未使能", foreground="gray")

            if msg.fault:
                self.lbl_fault.config(text="● 故障", foreground="red")
            else:
                self.lbl_fault.config(text="● 正常", foreground="gray")
        else:
            self.lbl_enabled.config(text="● 使能未知", foreground="gray")
            self.lbl_fault.config(text="● 故障未知", foreground="gray")

        if msg.feedback_fresh:
            self.feedback_text.set("新鲜")
            self.lbl_feedback.config(foreground="green")
        else:
            self.feedback_text.set("陈旧/未知")
            self.lbl_feedback.config(foreground="#b36b00")


class RealtimeMotorUI:
    def __init__(self) -> None:
        rospy.init_node("realtime_motor_ui", anonymous=True)

        self.node_cfg = self._load_node_cfg()
        self.joints = self._load_joints(self.node_cfg)
        if not self.joints:
            raise RuntimeError(
                f"未读取到 {NS}/joints 配置，且默认 can_driver.yaml 中也没有可用 joints"
            )
        self.can_device = self.joints[0]["can_device"]
        self.init_loopback = bool(rospy.get_param("~init_loopback", False))
        default_stream_hz = float(
            self.node_cfg.get(
                "control_frequency",
                rospy.get_param(f"{NS}/control_frequency", 20.0),
            )
        )
        self.stream_hz = max(1.0, float(rospy.get_param("~stream_hz", default_stream_hz)))
        self.stream_period_ms = max(4, int(round(1000.0 / self.stream_hz)))

        # --- 发布器 ---
        self.pub_pos = {}
        self.pub_vel = {}
        for j in self.joints:
            name = j["name"]
            self.pub_pos[name] = rospy.Publisher(
                f"{NS}/motor/{name}/cmd_position", Float64, queue_size=1
            )
            self.pub_vel[name] = rospy.Publisher(
                f"{NS}/motor/{name}/cmd_velocity", Float64, queue_size=1
            )

        # --- 服务代理 ---
        self.srv_motor_cmd = self._wait_service("motor_command", MotorCommand)
        self.srv_init = self._wait_service("init", Init)
        self.srv_shutdown = self._wait_service("shutdown", Shutdown)
        self.srv_recover = self._wait_service("recover", Recover)
        self.srv_set_zero_limit = self._wait_service("set_zero_limit", SetZeroLimit)
        self.srv_enable = self._wait_service("enable", Trigger)
        self.srv_disable = self._wait_service("disable", Trigger)
        self.srv_halt = self._wait_service("halt", Trigger)
        self.srv_resume = self._wait_service("resume", Trigger)

        # --- 订阅状态 ---
        self.motor_panels: dict[str, MotorPanel] = {}
        rospy.Subscriber(f"{NS}/motor_states", MotorState, self._on_motor_state)

        # --- UI ---
        self.root = tk.Tk()
        self.root.title("can_driver 多电机实时调试")
        self.root.geometry("900x700")
        self.root.resizable(True, True)

        self.selected_joint = tk.StringVar(value=self.joints[0]["name"])
        self.selected_mode = tk.StringVar(value=self.joints[0]["control_mode"])
        self.stream_enable = tk.BooleanVar(value=False)
        self.command_value = tk.DoubleVar(value=0.0)
        self.lifecycle_state = tk.StringVar(value="Configured/未知")
        self.link_mode = tk.StringVar(value="csp")
        self.link_stream_enable = tk.BooleanVar(value=False)
        self.link_command_value = tk.DoubleVar(value=0.0)
        self.zero_offset = tk.DoubleVar(value=0.0)
        self.limit_min = tk.DoubleVar(value=-1.0)
        self.limit_max = tk.DoubleVar(value=1.0)
        self.use_urdf_limits = tk.BooleanVar(value=False)
        self.apply_to_motor = tk.BooleanVar(value=True)
        self.zero_limit_result = tk.StringVar(value="零点/限位结果: --")
        self.status_text = tk.StringVar(
            value=(
                f"就绪 — 当前 CAN={self.can_device}, loopback={self.init_loopback}, "
                f"连续发送={self.stream_hz:.1f}Hz, joints来源={self.node_cfg.get('_source', 'param')}"
            )
        )

        self.pos_min = tk.DoubleVar(value=-3.14159)
        self.pos_max = tk.DoubleVar(value=3.14159)
        self.vel_min = tk.DoubleVar(value=-5.0)
        self.vel_max = tk.DoubleVar(value=5.0)
        self.link_pos_min = tk.DoubleVar(value=-3.14159)
        self.link_pos_max = tk.DoubleVar(value=3.14159)
        self.link_vel_min = tk.DoubleVar(value=-5.0)
        self.link_vel_max = tk.DoubleVar(value=5.0)
        self.link_joint_vars = {
            j["name"]: {
                "enabled": tk.BooleanVar(value=True),
                "scale": tk.DoubleVar(value=1.0),
                "offset": tk.DoubleVar(value=0.0),
            }
            for j in self.joints
        }
        self.link_preview_vars = {}

        self._build_ui()

        self.root.after(self.stream_period_ms, self._stream_loop)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------------------ load
    def _default_config_file(self):
        config_file = rospy.get_param("~config_file", "")
        if config_file:
            return config_file
        try:
            pkg_dir = rospkg.RosPack().get_path("can_driver")
            return str(Path(pkg_dir) / "config" / "can_driver.yaml")
        except rospkg.ResourceNotFound:
            return ""

    def _load_node_cfg(self):
        param_cfg = rospy.get_param(NS, None)
        if isinstance(param_cfg, dict) and param_cfg.get("joints"):
            cfg = dict(param_cfg)
            cfg["_source"] = "param"
            return cfg

        file_cfg, config_file = self._load_node_cfg_from_file()
        if file_cfg:
            if isinstance(param_cfg, dict):
                rospy.logwarn(
                    f"[UI] 参数服务器中的 {NS} 缺少可用 joints，已回退到配置文件: {config_file}"
                )
            else:
                rospy.logwarn(
                    f"[UI] 参数服务器中不存在 {NS}，已回退到配置文件: {config_file}"
                )
            return file_cfg

        if isinstance(param_cfg, dict):
            cfg = dict(param_cfg)
            cfg["_source"] = "param"
            return cfg
        return {"_source": "missing"}

    def _load_node_cfg_from_file(self):
        config_file = self._default_config_file()
        if not config_file:
            return None, ""

        try:
            with open(config_file, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            rospy.logwarn(f"[UI] 读取配置文件失败: {config_file} ({exc})")
            return None, config_file

        node_cfg = data.get("can_driver_node", {})
        if not isinstance(node_cfg, dict):
            return None, config_file

        cfg = dict(node_cfg)
        cfg["_source"] = Path(config_file).name
        return cfg, config_file

    def _load_joints(self, node_cfg):
        joints = node_cfg.get("joints", [])
        out = []
        for j in joints:
            out.append(
                {
                    "name": str(j.get("name", "")),
                    "motor_id": int(j.get("motor_id", 0)),
                    "protocol": str(j.get("protocol", "")),
                    "can_device": str(j.get("can_device", "can0")),
                    "control_mode": str(j.get("control_mode", "position")),
                    "position_scale": normalize_scale(j.get("position_scale", 1.0)),
                    "velocity_scale": normalize_scale(j.get("velocity_scale", 1.0)),
                }
            )
        return [j for j in out if j["name"]]

    def _wait_service(self, name, srv_type, timeout=5.0):
        full = f"{NS}/{name}"
        try:
            rospy.wait_for_service(full, timeout=timeout)
        except rospy.ROSException:
            rospy.logwarn(f"[UI] 服务 {full} 等待超时，继续启动")
        return rospy.ServiceProxy(full, srv_type)

    # --------------------------------------------------------------- build UI
    def _build_ui(self) -> None:
        # ======== 生命周期按钮 ========
        frm_lifecycle = ttk.LabelFrame(self.root, text="生命周期", padding=6)
        frm_lifecycle.pack(fill=tk.X, padx=8, pady=(8, 4))

        lifecycle_btns = [
            ("Init", self._lifecycle_init),
            ("Init + Release", self._lifecycle_init_release),
            ("Enable", self._lifecycle_enable),
            ("Disable", self._lifecycle_disable),
            ("Release/Resume", self._lifecycle_resume),
            ("Halt", self._lifecycle_halt),
            ("Recover", self._lifecycle_recover),
            ("Shutdown", self._lifecycle_shutdown),
        ]
        for i, (text, cmd) in enumerate(lifecycle_btns):
            ttk.Button(frm_lifecycle, text=text, command=cmd).grid(
                row=0, column=i, padx=4, pady=2
            )
        ttk.Label(frm_lifecycle, text="状态:").grid(row=1, column=0, sticky="w", padx=4, pady=(4, 0))
        ttk.Label(frm_lifecycle, textvariable=self.lifecycle_state, foreground="dark green").grid(
            row=1, column=1, columnspan=3, sticky="w", padx=4, pady=(4, 0)
        )

        # ======== 电机状态面板 ========
        frm_states = ttk.LabelFrame(self.root, text="电机实时状态", padding=6)
        frm_states.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

        canvas = tk.Canvas(frm_states, highlightthickness=0)
        scrollbar = ttk.Scrollbar(frm_states, orient=tk.VERTICAL, command=canvas.yview)
        self.frm_motors = ttk.Frame(canvas)

        self.frm_motors.bind(
            "<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        canvas.create_window((0, 0), window=self.frm_motors, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        for j in self.joints:
            panel = MotorPanel(self.frm_motors, j, self.srv_motor_cmd)
            panel.frame.pack(fill=tk.X, padx=4, pady=2)
            self.motor_panels[j["name"]] = panel

        # ======== 命令控制区 ========
        frm_cmd = ttk.LabelFrame(self.root, text="实时命令", padding=6)
        frm_cmd.pack(fill=tk.X, padx=8, pady=4)

        # 关节选择 + 模式
        row1 = ttk.Frame(frm_cmd)
        row1.pack(fill=tk.X, pady=2)

        ttk.Label(row1, text="目标关节").pack(side=tk.LEFT)
        joint_names = [j["name"] for j in self.joints]
        cmb = ttk.Combobox(
            row1,
            textvariable=self.selected_joint,
            values=joint_names,
            state="readonly",
            width=18,
        )
        cmb.pack(side=tk.LEFT, padx=6)
        cmb.bind(
            "<<ComboboxSelected>>",
            lambda _: self._on_joint_changed(),
        )

        ttk.Label(row1, text="模式").pack(side=tk.LEFT, padx=(12, 0))
        for label, val in [("位置", "position"), ("速度", "velocity"), ("CSP", "csp")]:
            ttk.Radiobutton(
                row1,
                text=label,
                value=val,
                variable=self.selected_mode,
                command=self._apply_range,
            ).pack(side=tk.LEFT, padx=4)

        ttk.Checkbutton(
            row1, text="连续发送", variable=self.stream_enable
        ).pack(side=tk.LEFT, padx=12)

        # 滑条
        self.scale = tk.Scale(
            frm_cmd,
            from_=self.pos_min.get(),
            to=self.pos_max.get(),
            resolution=0.001,
            orient=tk.HORIZONTAL,
            length=700,
            variable=self.command_value,
            command=self._on_slider_changed,
        )
        self.scale.pack(fill=tk.X, pady=4)

        # 范围
        row_range = ttk.Frame(frm_cmd)
        row_range.pack(fill=tk.X, pady=2)
        ttk.Label(row_range, text="位置范围").pack(side=tk.LEFT)
        ttk.Entry(row_range, width=8, textvariable=self.pos_min).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Entry(row_range, width=8, textvariable=self.pos_max).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Label(row_range, text="速度范围").pack(side=tk.LEFT, padx=(12, 0))
        ttk.Entry(row_range, width=8, textvariable=self.vel_min).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Entry(row_range, width=8, textvariable=self.vel_max).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(row_range, text="应用", command=self._apply_range).pack(
            side=tk.LEFT, padx=8
        )

        # 单电机按钮
        row_motor = ttk.Frame(frm_cmd)
        row_motor.pack(fill=tk.X, pady=2)
        ttk.Button(row_motor, text="单电机使能", command=self._motor_enable).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Button(row_motor, text="单电机停止", command=self._motor_stop).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Button(row_motor, text="单电机失能", command=self._motor_disable).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Button(row_motor, text="应用模式", command=self._apply_selected_mode).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Button(row_motor, text="发送一次", command=self._publish_once).pack(
            side=tk.LEFT, padx=4
        )

        # 零点/限位
        frm_zero = ttk.LabelFrame(self.root, text="零点/限位（当前选中关节）", padding=6)
        frm_zero.pack(fill=tk.X, padx=8, pady=4)

        row_zero1 = ttk.Frame(frm_zero)
        row_zero1.pack(fill=tk.X, pady=2)
        ttk.Label(row_zero1, text="偏置(rad)").pack(side=tk.LEFT)
        ttk.Entry(row_zero1, width=10, textvariable=self.zero_offset).pack(side=tk.LEFT, padx=4)
        ttk.Label(row_zero1, text="下限(rad)").pack(side=tk.LEFT, padx=(8, 0))
        ttk.Entry(row_zero1, width=10, textvariable=self.limit_min).pack(side=tk.LEFT, padx=4)
        ttk.Label(row_zero1, text="上限(rad)").pack(side=tk.LEFT, padx=(8, 0))
        ttk.Entry(row_zero1, width=10, textvariable=self.limit_max).pack(side=tk.LEFT, padx=4)

        row_zero2 = ttk.Frame(frm_zero)
        row_zero2.pack(fill=tk.X, pady=2)
        ttk.Checkbutton(row_zero2, text="使用URDF限位", variable=self.use_urdf_limits).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Checkbutton(row_zero2, text="下发到电机", variable=self.apply_to_motor).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Button(row_zero2, text="应用零点/限位", command=self._apply_zero_limit).pack(
            side=tk.LEFT, padx=8
        )
        ttk.Label(row_zero2, textvariable=self.zero_limit_result, foreground="#0b5d86").pack(
            side=tk.LEFT, padx=8
        )

        # ======== 多轴联动区 ========
        frm_link = ttk.LabelFrame(self.root, text="多轴联动", padding=6)
        frm_link.pack(fill=tk.X, padx=8, pady=4)

        row_link_mode = ttk.Frame(frm_link)
        row_link_mode.pack(fill=tk.X, pady=2)
        ttk.Label(row_link_mode, text="联动模式").pack(side=tk.LEFT)
        for label, val in [("位置", "position"), ("速度", "velocity"), ("CSP", "csp")]:
            ttk.Radiobutton(
                row_link_mode,
                text=label,
                value=val,
                variable=self.link_mode,
                command=self._apply_link_range,
            ).pack(side=tk.LEFT, padx=4)

        ttk.Checkbutton(
            row_link_mode, text="联动连续发送", variable=self.link_stream_enable
        ).pack(side=tk.LEFT, padx=12)
        ttk.Button(
            row_link_mode, text="联动切模式", command=self._link_set_mode
        ).pack(side=tk.LEFT, padx=4)
        ttk.Button(
            row_link_mode, text="联动发送一次", command=self._link_publish_once
        ).pack(side=tk.LEFT, padx=4)

        self.link_scale = tk.Scale(
            frm_link,
            from_=self.link_pos_min.get(),
            to=self.link_pos_max.get(),
            resolution=0.001,
            orient=tk.HORIZONTAL,
            length=700,
            variable=self.link_command_value,
            command=self._on_link_slider_changed,
        )
        self.link_scale.pack(fill=tk.X, pady=4)

        row_link_range = ttk.Frame(frm_link)
        row_link_range.pack(fill=tk.X, pady=2)
        ttk.Label(row_link_range, text="位置范围").pack(side=tk.LEFT)
        ttk.Entry(row_link_range, width=8, textvariable=self.link_pos_min).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Entry(row_link_range, width=8, textvariable=self.link_pos_max).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Label(row_link_range, text="速度范围").pack(side=tk.LEFT, padx=(12, 0))
        ttk.Entry(row_link_range, width=8, textvariable=self.link_vel_min).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Entry(row_link_range, width=8, textvariable=self.link_vel_max).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(row_link_range, text="应用", command=self._apply_link_range).pack(
            side=tk.LEFT, padx=8
        )
        ttk.Button(
            row_link_range, text="全选", command=lambda: self._set_link_selection(True)
        ).pack(side=tk.LEFT, padx=4)
        ttk.Button(
            row_link_range, text="清空", command=lambda: self._set_link_selection(False)
        ).pack(side=tk.LEFT, padx=4)

        row_link_motor = ttk.Frame(frm_link)
        row_link_motor.pack(fill=tk.X, pady=2)
        ttk.Button(
            row_link_motor, text="联动使能", command=self._link_enable
        ).pack(side=tk.LEFT, padx=4)
        ttk.Button(
            row_link_motor, text="联动停止", command=self._link_stop
        ).pack(side=tk.LEFT, padx=4)
        ttk.Button(
            row_link_motor, text="联动失能", command=self._link_disable
        ).pack(side=tk.LEFT, padx=4)

        row_link_header = ttk.Frame(frm_link)
        row_link_header.pack(fill=tk.X, pady=(6, 2))
        ttk.Label(row_link_header, text="联动关节", width=20).pack(side=tk.LEFT)
        ttk.Label(row_link_header, text="倍率", width=10).pack(side=tk.LEFT)
        ttk.Label(row_link_header, text="偏置", width=10).pack(side=tk.LEFT)

        for joint in self.joints:
            row_joint = ttk.Frame(frm_link)
            row_joint.pack(fill=tk.X, pady=1)
            vars_for_joint = self.link_joint_vars[joint["name"]]
            ttk.Checkbutton(
                row_joint,
                text=f"{joint['name']} (0x{joint['motor_id']:02X})",
                variable=vars_for_joint["enabled"],
                command=self._refresh_link_preview,
            ).pack(side=tk.LEFT)
            ttk.Entry(row_joint, width=10, textvariable=vars_for_joint["scale"]).pack(
                side=tk.LEFT, padx=(8, 4)
            )
            ttk.Entry(row_joint, width=10, textvariable=vars_for_joint["offset"]).pack(
                side=tk.LEFT, padx=4
            )
            preview = tk.StringVar(value="目标=0.0000")
            self.link_preview_vars[joint["name"]] = preview
            ttk.Label(row_joint, textvariable=preview, width=18).pack(
                side=tk.LEFT, padx=8
            )

        # 状态栏
        frm_status = ttk.Frame(self.root, padding=4)
        frm_status.pack(fill=tk.X, padx=8, pady=(0, 4))
        ttk.Label(frm_status, textvariable=self.status_text, foreground="blue").pack(
            side=tk.LEFT
        )

        self._apply_link_range()
        self._refresh_link_preview()

    # ------------------------------------------------------- lifecycle actions
    def _call_async(self, label, func):
        """在后台线程调用服务，避免阻塞 UI"""
        self.status_text.set(f"{label} ...")

        def _run():
            try:
                res = func()
                msg = getattr(res, "message", "OK") if res.success else f"失败: {res.message}"
                self.root.after(0, lambda: self.status_text.set(f"{label}: {msg}"))
            except Exception as exc:
                self.root.after(
                    0, lambda: self.status_text.set(f"{label} 异常: {exc}")
                )

        threading.Thread(target=_run, daemon=True).start()

    def _set_lifecycle_state(self, state: str) -> None:
        self.lifecycle_state.set(state)

    def _lifecycle_init_release(self):
        self.status_text.set(
            f"Init({self.can_device}, loopback={self.init_loopback}) + Release ..."
        )

        def _run():
            try:
                res1 = self.srv_init(device=self.can_device, loopback=self.init_loopback)
                if not res1.success:
                    self.root.after(
                        0,
                        lambda: self.status_text.set(f"Init 失败: {res1.message}"),
                    )
                    return
                res2 = self.srv_resume()
                msg = res2.message if res2.success else f"Release 失败: {res2.message}"
                self.root.after(
                    0,
                    lambda: (
                        self.status_text.set(f"Init + Release: {msg}"),
                        self._set_lifecycle_state("Running" if res2.success else "Armed")
                    ),
                )
            except Exception as exc:
                self.root.after(
                    0, lambda: self.status_text.set(f"Init+Release 异常: {exc}")
                )

        threading.Thread(target=_run, daemon=True).start()

    def _lifecycle_init(self):
        def _run():
            res = self.srv_init(device=self.can_device, loopback=self.init_loopback)
            if res.success:
                self.root.after(0, lambda: self._set_lifecycle_state("Armed"))
            return res

        self._call_async("Init", _run)

    def _lifecycle_enable(self):
        def _run():
            res = self.srv_enable()
            if res.success:
                self.root.after(0, lambda: self._set_lifecycle_state("Armed"))
            return res

        self._call_async("Enable", _run)

    def _lifecycle_disable(self):
        def _run():
            res = self.srv_disable()
            if res.success:
                self.root.after(0, lambda: self._set_lifecycle_state("Standby"))
            return res

        self._call_async("Disable", _run)

    def _lifecycle_halt(self):
        def _run():
            res = self.srv_halt()
            if res.success:
                self.root.after(0, lambda: self._set_lifecycle_state("Armed"))
            return res

        self._call_async("Halt", _run)

    def _lifecycle_resume(self):
        def _run():
            res = self.srv_resume()
            if res.success:
                self.root.after(0, lambda: self._set_lifecycle_state("Running"))
            return res

        self._call_async("Resume", _run)

    def _lifecycle_recover(self):
        def _run():
            res = self.srv_recover(motor_id=0xFFFF)
            if res.success:
                self.root.after(0, lambda: self._set_lifecycle_state("Standby"))
            return res

        self._call_async("Recover", _run)

    def _lifecycle_shutdown(self):
        def _run():
            res = self.srv_shutdown(force=False)
            if res.success:
                self.root.after(0, lambda: self._set_lifecycle_state("Configured"))
            return res

        self._call_async("Shutdown", _run)

    # --------------------------------------------------- motor state callback
    def _on_motor_state(self, msg: MotorState):
        name = msg.name
        if name in self.motor_panels:
            self.root.after(0, lambda: self.motor_panels[name].update_state(msg))

    # -------------------------------------------------------- command helpers
    def _get_joint_cfg(self, joint_name: str):
        for j in self.joints:
            if j["name"] == joint_name:
                return j
        raise RuntimeError(f"未知关节: {joint_name}")

    def _send_motor_service(self, command: int, value: float = 0.0) -> None:
        cfg = self._get_joint_cfg(self.selected_joint.get())
        self._send_motor_service_for_cfg(cfg, command, value)

    def _send_motor_service_for_cfg(
        self, cfg: dict, command: int, value: float = 0.0
    ) -> None:
        label = f"{cfg['name']} 指令"
        self.status_text.set(f"{label} ...")

        def _run():
            try:
                res = self.srv_motor_cmd(
                    motor_id=cfg["motor_id"], command=command, value=value
                )
                def _finish():
                    self.status_text.set(
                        res.message if res.success else f"失败: {res.message}"
                    )
                    if res.success and command == CMD_SET_MODE:
                        cfg["control_mode"] = self.selected_mode.get()
                self.root.after(0, _finish)
            except Exception as exc:
                self.root.after(0, lambda: self.status_text.set(f"服务调用异常: {exc}"))

        threading.Thread(target=_run, daemon=True).start()

    def _on_joint_changed(self) -> None:
        name = self.selected_joint.get()
        cfg = self._get_joint_cfg(name)
        self.selected_mode.set(cfg["control_mode"])
        self._apply_range()
        self.status_text.set(f"已选择: {name} (ID=0x{cfg['motor_id']:02X})")

    def _apply_selected_mode(self) -> None:
        mode = self.selected_mode.get()
        val = MODE_MAP.get(mode, 0.0)
        self._send_motor_service(CMD_SET_MODE, val)
        self._apply_range()

    def _apply_range(self) -> None:
        mode = self.selected_mode.get()
        if mode == "velocity":
            self.scale.configure(
                from_=self.vel_min.get(), to=self.vel_max.get(), resolution=0.001
            )
        else:
            self.scale.configure(
                from_=self.pos_min.get(), to=self.pos_max.get(), resolution=0.001
            )

    def _apply_link_range(self) -> None:
        mode = self.link_mode.get()
        if mode == "velocity":
            self.link_scale.configure(
                from_=self.link_vel_min.get(),
                to=self.link_vel_max.get(),
                resolution=0.001,
            )
        else:
            self.link_scale.configure(
                from_=self.link_pos_min.get(),
                to=self.link_pos_max.get(),
                resolution=0.001,
            )
        self._refresh_link_preview()

    def _motor_enable(self) -> None:
        self._send_motor_service(CMD_ENABLE)

    def _motor_stop(self) -> None:
        self._send_motor_service(CMD_STOP)

    def _motor_disable(self) -> None:
        self._send_motor_service(CMD_DISABLE)

    def _apply_zero_limit(self) -> None:
        cfg = self._get_joint_cfg(self.selected_joint.get())
        self.status_text.set(f"{cfg['name']} 零点/限位应用中 ...")

        def _run():
            try:
                res = self.srv_set_zero_limit(
                    motor_id=cfg["motor_id"],
                    zero_offset_rad=float(self.zero_offset.get()),
                    min_position_rad=float(self.limit_min.get()),
                    max_position_rad=float(self.limit_max.get()),
                    use_urdf_limits=bool(self.use_urdf_limits.get()),
                    apply_to_motor=bool(self.apply_to_motor.get()),
                )

                def _finish():
                    if res.success:
                        self.status_text.set(
                            f"{cfg['name']} 零点/限位已应用: {res.message}"
                        )
                        self.zero_limit_result.set(
                            "零点/限位结果: "
                            f"pos={res.current_position_rad:.4f}, "
                            f"[{res.applied_min_rad:.4f}, {res.applied_max_rad:.4f}]"
                        )
                    else:
                        self.status_text.set(f"{cfg['name']} 零点/限位失败: {res.message}")
                        self.zero_limit_result.set(f"零点/限位结果: 失败 ({res.message})")

                self.root.after(0, _finish)
            except Exception as exc:
                self.root.after(
                    0,
                    lambda: (
                        self.status_text.set(f"{cfg['name']} 零点/限位异常: {exc}"),
                        self.zero_limit_result.set(f"零点/限位结果: 异常 ({exc})"),
                    ),
                )

        threading.Thread(target=_run, daemon=True).start()

    def _selected_link_joints(self):
        selected = []
        for joint in self.joints:
            vars_for_joint = self.link_joint_vars[joint["name"]]
            if vars_for_joint["enabled"].get():
                selected.append(joint)
        return selected

    def _refresh_link_preview(self) -> None:
        base_value = float(self.link_command_value.get())
        for joint in self.joints:
            vars_for_joint = self.link_joint_vars[joint["name"]]
            target = (
                base_value * float(vars_for_joint["scale"].get())
                + float(vars_for_joint["offset"].get())
            )
            preview = self.link_preview_vars.get(joint["name"])
            if preview is not None:
                preview.set(f"目标={target:.4f}")

    def _set_link_selection(self, enabled: bool) -> None:
        for vars_for_joint in self.link_joint_vars.values():
            vars_for_joint["enabled"].set(enabled)
        self._refresh_link_preview()
        self.status_text.set(f"联动关节数量: {len(self._selected_link_joints())}")

    def _call_group_motor_service(
        self, label: str, command: int, value: float = 0.0
    ) -> None:
        selected = list(self._selected_link_joints())
        if not selected:
            self.status_text.set(f"{label} 已跳过: 未勾选任何关节")
            return

        self.status_text.set(f"{label} ...")

        def _run():
            failed = []
            for cfg in selected:
                try:
                    res = self.srv_motor_cmd(
                        motor_id=cfg["motor_id"], command=command, value=value
                    )
                    if not res.success:
                        failed.append(f"{cfg['name']}:{res.message}")
                except Exception as exc:
                    failed.append(f"{cfg['name']}:{exc}")

            if failed:
                message = f"{label} 部分失败: " + "; ".join(failed[:4])
            else:
                message = f"{label} 完成: {len(selected)} 轴"
            self.root.after(0, lambda: self.status_text.set(message))

        threading.Thread(target=_run, daemon=True).start()

    def _link_set_mode(self) -> None:
        mode = self.link_mode.get()
        self._call_group_motor_service("联动切模式", CMD_SET_MODE, MODE_MAP.get(mode, 0.0))

    def _link_enable(self) -> None:
        self._call_group_motor_service("联动使能", CMD_ENABLE)

    def _link_stop(self) -> None:
        self._call_group_motor_service("联动停止", CMD_STOP)

    def _link_disable(self) -> None:
        self._call_group_motor_service("联动失能", CMD_DISABLE)

    # -------------------------------------------------------- publish command
    def _publish_once(self) -> None:
        name = self.selected_joint.get()
        val = float(self.command_value.get())
        cfg = self._get_joint_cfg(name)
        self._publish_joint_command(cfg, self.selected_mode.get(), val)

    def _publish_joint_command(self, cfg: dict, mode: str, value: float) -> None:
        msg = Float64(data=value)
        if mode == "velocity":
            self.pub_vel[cfg["name"]].publish(msg)
        else:
            self.pub_pos[cfg["name"]].publish(msg)

    def _link_publish_once(self) -> None:
        selected = self._selected_link_joints()
        if not selected:
            self.status_text.set("联动发送已跳过: 未勾选任何关节")
            return

        base_value = float(self.link_command_value.get())
        mode = self.link_mode.get()
        preview_parts = []
        for joint in selected:
            vars_for_joint = self.link_joint_vars[joint["name"]]
            target = (
                base_value * float(vars_for_joint["scale"].get())
                + float(vars_for_joint["offset"].get())
            )
            self._publish_joint_command(joint, mode, target)
            preview_parts.append(f"{joint['name']}={target:.3f}")

        unit = "rad/s" if mode == "velocity" else "rad"
        self.status_text.set(
            f"联动发送[{mode}] {len(selected)}轴: "
            + ", ".join(preview_parts[:4])
            + f" {unit}"
        )

    def _on_slider_changed(self, _value: str) -> None:
        if not self.stream_enable.get():
            return
        try:
            self._publish_once()
            name = self.selected_joint.get()
            val = float(self.command_value.get())
            mode = self.selected_mode.get()
            unit = "rad/s" if mode == "velocity" else "rad"
            self.status_text.set(f"→ {name}: {val:.4f} {unit}")
        except Exception as exc:
            self.status_text.set(f"发布异常: {exc}")

    def _on_link_slider_changed(self, _value: str) -> None:
        self._refresh_link_preview()
        if not self.link_stream_enable.get():
            return
        try:
            self._link_publish_once()
        except Exception as exc:
            self.status_text.set(f"联动发布异常: {exc}")

    def _stream_loop(self) -> None:
        if self.stream_enable.get():
            try:
                self._publish_once()
            except Exception:
                pass
        if self.link_stream_enable.get():
            try:
                self._link_publish_once()
            except Exception:
                pass
        self.root.after(self.stream_period_ms, self._stream_loop)

    # -------------------------------------------------------------- shutdown
    def _on_close(self) -> None:
        try:
            for j in self.joints:
                self.srv_motor_cmd(
                    motor_id=j["motor_id"], command=CMD_STOP, value=0.0
                )
        except Exception:
            pass
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


if __name__ == "__main__":
    try:
        ui = RealtimeMotorUI()
        ui.run()
    except Exception as e:
        rospy.logerr("[realtime_motor_ui] 启动失败: %s", e)
        raise
