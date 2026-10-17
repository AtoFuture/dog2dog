#!/usr/bin/env python3
# Author: lnotspotl, abutalipovvv
# NOTE(2026-10-13): 修复 PID 时间源。原实现用 rclpy.time.Time()（无 clock 节点）恒为 0，
# 导致 step<1e-6 每帧直接 return [0,0] —— 即 IMU 姿态补偿从未生效。改用单调时钟。
import time
import numpy as np


class PID_controller:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.desired_roll_pitch = np.array([0.0, 0.0])

        self.I_term = np.array([0.0, 0.0])
        self.D_term = np.array([0.0, 0.0])

        self.max_I = 0.2
        self.last_error = np.array([0.0, 0.0])
        self.last_time = None

    def run(self, roll, pitch):
        error = self.desired_roll_pitch - np.array([roll, pitch])

        t_now = self.get_time_in_seconds()
        if self.last_time is None:
            self.last_time = t_now
            return np.array([0.0, 0.0])

        step = t_now - self.last_time
        if step < 1e-6:
            return np.array([0.0, 0.0])
        if step > 0.2:          # 防止长时间停帧后 D 项爆炸
            step = 0.2

        self.I_term += error * step
        for i in range(2):
            if self.I_term[i] < -self.max_I:
                self.I_term[i] = -self.max_I
            elif self.I_term[i] > self.max_I:
                self.I_term[i] = self.max_I

        self.D_term = (error - self.last_error) / step

        self.last_time = t_now
        self.last_error = error

        return self.kp * error + self.I_term * self.ki + self.D_term * self.kd

    def reset(self):
        self.last_time = self.get_time_in_seconds()
        self.I_term = np.array([0.0, 0.0])
        self.D_term = np.array([0.0, 0.0])
        self.last_error = np.array([0.0, 0.0])

    def set_desired_RP_angles(self, des_roll, des_pitch):
        self.desired_roll_pitch = np.array([des_roll, des_pitch])

    def get_time_in_seconds(self):
        return time.monotonic()
