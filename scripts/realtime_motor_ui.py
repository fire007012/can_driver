#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import tkinter as tk
from tkinter import ttk

import rospy
from std_msgs.msg import Float64

from can_driver.srv import MotorCommand


CMD_ENABLE = 0
CMD_DISABLE = 1
CMD_STOP = 2
CMD_SET_MODE = 3


class RealtimeMotorUI:
    def __init__(self) -> None:
        rospy.init_node("realtime_motor_ui", anonymous=True)

        self.joints = self._load_joints()
        if not self.joints:
            raise RuntimeError("未读取到 /can_driver_node/joints 配置")

        self.pub_pos = {}
        self.pub_vel = {}
        for j in self.joints:
            name = j["name"]
            self.pub_pos[name] = rospy.Publisher(
                f"/can_driver_node/motor/{name}/cmd_position", Float64, queue_size=1
            )
            self.pub_vel[name] = rospy.Publisher(
                f"/can_driver_node/motor/{name}/cmd_velocity", Float64, queue_size=1
            )

        rospy.wait_for_service("/can_driver_node/motor_command", timeout=3.0)
        self.motor_cmd = rospy.ServiceProxy("/can_driver_node/motor_command", MotorCommand)

        self.root = tk.Tk()
        self.root.title("can_driver 实时电机控制")
        self.root.geometry("720x380")

        self.selected_joint = tk.StringVar(value=self.joints[0]["name"])
        self.selected_mode = tk.StringVar(value=self.joints[0]["control_mode"])
        self.stream_enable = tk.BooleanVar(value=True)

        self.pos_min = tk.DoubleVar(value=-3.14159)
        self.pos_max = tk.DoubleVar(value=3.14159)
        self.vel_min = tk.DoubleVar(value=-5.0)
        self.vel_max = tk.DoubleVar(value=5.0)

        self.command_value = tk.DoubleVar(value=0.0)
        self.status_text = tk.StringVar(value="就绪")

        self._build_ui()
        self._on_joint_changed(self.selected_joint.get())

        self.root.after(50, self._stream_loop)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _load_joints(self):
        joints = rospy.get_param("/can_driver_node/joints", [])
        out = []
        for j in joints:
            out.append(
                {
                    "name": str(j.get("name", "")),
                    "motor_id": int(j.get("motor_id", 0)),
                    "protocol": str(j.get("protocol", "")),
                    "control_mode": str(j.get("control_mode", "position")),
                }
            )
        return [j for j in out if j["name"]]

    def _build_ui(self) -> None:
        frm_top = ttk.Frame(self.root, padding=12)
        frm_top.pack(fill=tk.X)

        ttk.Label(frm_top, text="关节").grid(row=0, column=0, sticky=tk.W)
        joint_names = [j["name"] for j in self.joints]
        cmb = ttk.Combobox(
            frm_top,
            textvariable=self.selected_joint,
            values=joint_names,
            state="readonly",
            width=22,
        )
        cmb.grid(row=0, column=1, padx=8, sticky=tk.W)
        cmb.bind("<<ComboboxSelected>>", lambda _e: self._on_joint_changed(self.selected_joint.get()))

        self.info_label = ttk.Label(frm_top, text="")
        self.info_label.grid(row=0, column=2, padx=12, sticky=tk.W)

        frm_mode = ttk.LabelFrame(self.root, text="控制模式", padding=12)
        frm_mode.pack(fill=tk.X, padx=12, pady=8)

        ttk.Radiobutton(
            frm_mode,
            text="位置模式",
            value="position",
            variable=self.selected_mode,
            command=self._on_mode_changed,
        ).pack(side=tk.LEFT, padx=8)

        ttk.Radiobutton(
            frm_mode,
            text="速度模式",
            value="velocity",
            variable=self.selected_mode,
            command=self._on_mode_changed,
        ).pack(side=tk.LEFT, padx=8)

        ttk.Checkbutton(
            frm_mode,
            text="实时连续发送",
            variable=self.stream_enable,
        ).pack(side=tk.LEFT, padx=18)

        frm_ctrl = ttk.LabelFrame(self.root, text="实时命令", padding=12)
        frm_ctrl.pack(fill=tk.BOTH, expand=True, padx=12, pady=8)

        self.scale = tk.Scale(
            frm_ctrl,
            from_=self.pos_min.get(),
            to=self.pos_max.get(),
            resolution=0.001,
            orient=tk.HORIZONTAL,
            length=620,
            variable=self.command_value,
            command=self._on_slider_changed,
        )
        self.scale.pack(fill=tk.X, pady=10)

        frm_range = ttk.Frame(frm_ctrl)
        frm_range.pack(fill=tk.X)
        ttk.Label(frm_range, text="位置范围").grid(row=0, column=0, sticky=tk.W)
        ttk.Entry(frm_range, width=8, textvariable=self.pos_min).grid(row=0, column=1, padx=4)
        ttk.Entry(frm_range, width=8, textvariable=self.pos_max).grid(row=0, column=2, padx=4)

        ttk.Label(frm_range, text="速度范围").grid(row=0, column=3, padx=(16, 0), sticky=tk.W)
        ttk.Entry(frm_range, width=8, textvariable=self.vel_min).grid(row=0, column=4, padx=4)
        ttk.Entry(frm_range, width=8, textvariable=self.vel_max).grid(row=0, column=5, padx=4)

        ttk.Button(frm_range, text="应用范围", command=self._apply_range).grid(row=0, column=6, padx=10)

        frm_btn = ttk.Frame(self.root, padding=12)
        frm_btn.pack(fill=tk.X)

        ttk.Button(frm_btn, text="使能", command=self._enable).pack(side=tk.LEFT, padx=6)
        ttk.Button(frm_btn, text="停止", command=self._stop).pack(side=tk.LEFT, padx=6)
        ttk.Button(frm_btn, text="失能", command=self._disable).pack(side=tk.LEFT, padx=6)

        ttk.Label(frm_btn, textvariable=self.status_text).pack(side=tk.LEFT, padx=16)

    def _get_joint_cfg(self, joint_name: str):
        for j in self.joints:
            if j["name"] == joint_name:
                return j
        raise RuntimeError(f"未知关节: {joint_name}")

    def _on_joint_changed(self, joint_name: str) -> None:
        cfg = self._get_joint_cfg(joint_name)
        self.info_label.config(
            text=f"motor_id={cfg['motor_id']} | protocol={cfg['protocol']} | 默认模式={cfg['control_mode']}"
        )
        self.selected_mode.set(cfg["control_mode"])
        self._apply_range()
        self.status_text.set(f"已切换关节: {joint_name}")

    def _apply_range(self) -> None:
        if self.selected_mode.get() == "position":
            self.scale.configure(from_=self.pos_min.get(), to=self.pos_max.get(), resolution=0.001)
        else:
            self.scale.configure(from_=self.vel_min.get(), to=self.vel_max.get(), resolution=0.001)

    def _send_service(self, command: int, value: float = 0.0) -> None:
        cfg = self._get_joint_cfg(self.selected_joint.get())
        res = self.motor_cmd(motor_id=cfg["motor_id"], command=command, value=value)
        self.status_text.set(res.message if res.success else f"失败: {res.message}")

    def _on_mode_changed(self) -> None:
        mode = self.selected_mode.get()
        value = 0.0 if mode == "position" else 1.0
        try:
            self._send_service(CMD_SET_MODE, value)
        except Exception as exc:
            self.status_text.set(f"切模式异常: {exc}")
        self._apply_range()

    def _enable(self) -> None:
        try:
            self._send_service(CMD_ENABLE, 0.0)
        except Exception as exc:
            self.status_text.set(f"使能异常: {exc}")

    def _disable(self) -> None:
        try:
            self._send_service(CMD_DISABLE, 0.0)
        except Exception as exc:
            self.status_text.set(f"失能异常: {exc}")

    def _stop(self) -> None:
        try:
            self._send_service(CMD_STOP, 0.0)
        except Exception as exc:
            self.status_text.set(f"停止异常: {exc}")

    def _publish_once(self) -> None:
        name = self.selected_joint.get()
        val = float(self.command_value.get())
        msg = Float64(data=val)
        if self.selected_mode.get() == "position":
            self.pub_pos[name].publish(msg)
            self.status_text.set(f"位置命令: {name} -> {val:.4f} rad")
        else:
            self.pub_vel[name].publish(msg)
            self.status_text.set(f"速度命令: {name} -> {val:.4f} rad/s")

    def _on_slider_changed(self, _value: str) -> None:
        try:
            self._publish_once()
        except Exception as exc:
            self.status_text.set(f"发布异常: {exc}")

    def _stream_loop(self) -> None:
        if self.stream_enable.get():
            try:
                self._publish_once()
            except Exception:
                pass
        self.root.after(50, self._stream_loop)

    def _on_close(self) -> None:
        try:
            self._stop()
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
