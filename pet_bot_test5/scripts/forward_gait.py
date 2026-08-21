#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# ============================================================
# ROBOT GEOMETRY
# ============================================================
L1 = 0.08
L2 = 0.08

# ============================================================
# GAIT
# ============================================================
RATE = 50.0
CYCLE_TIME = 0.8
STEP_LENGTH = 0.07
STEP_HEIGHT = 0.02
BODY_HEIGHT = 0.15

DX_STANCE = STEP_LENGTH / (CYCLE_TIME / 2) / RATE

STARTUP_HOLD = 1.0

LEGS = ["FL", "FR", "RL", "RR"]
LEG_INDEX = {"FL": 0, "FR": 3, "RL": 6, "RR": 9}

SWING_A = ["FL", "RR"]
SWING_B = ["FR", "RL"]

# ============================================================
# NEUTRAL (STRAIGHT)
# ============================================================
NEUTRAL = {
    "FL": {"hip": -0.4, "thigh": 0.0, "knee": 0.0},
    "FR": {"hip": 0.4, "thigh": 0.0, "knee": 0.0},
    "RL": {"hip": 0.4, "thigh": 0.0, "knee": 0.0},
    "RR": {"hip": -0.4, "thigh": 0.0, "knee": 0.0},
}
HIP_SIGN = {"FL": -1, "FR": +1, "RL": +1, "RR": -1}
THIGH_SIGN = {"FL": +1, "FR": -1, "RL": -1, "RR": -1}
KNEE_SIGN  = {"FL": +1, "FR": -1, "RL": +1, "RR": -1}

# ============================================================
# IK
# ============================================================
def leg_ik(x, z):
    r = math.hypot(x, z)
    r = max(min(r, L1 + L2 - 1e-6), abs(L1 - L2) + 1e-6)

    cos_k = (L1*L1 + L2*L2 - r*r) / (2*L1*L2)
    knee = -(math.pi - math.acos(cos_k))

    phi = math.atan2(z, x)
    cos_a = (L1*L1 + r*r - L2*L2) / (2*L1*r)
    alpha = math.acos(cos_a)

    thigh = phi + alpha - math.pi/2
    return thigh, knee

# ============================================================
# FOOT TRAJECTORY WITH WORLD STANCE
# ============================================================
def foot_position(s, swing):
    if swing:
        x = -STEP_LENGTH/2 + STEP_LENGTH * s + 0.04
        z = (BODY_HEIGHT-STEP_HEIGHT* (math.sin(math.pi * s))**2)
    else:
        x = STEP_LENGTH/2 - STEP_LENGTH * s -0.02
        z = BODY_HEIGHT 
    return x, z

# ============================================================
# ROS NODE
# ============================================================
class HybridIKTrot(Node):

    def __init__(self,steps):
        super().__init__("hybrid_ik_trot_world")

        self.pub = self.create_publisher(
            Float64MultiArray,
            "/forward_position_controller/commands",
            10
        )

        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.timer = self.create_timer(1.0 / RATE, self.update)

        self.prev_phase = 0.0
        self.cycles = 0
        self.max_cycles = steps

        self.get_logger().info("Forward motion started")

    def update(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        t = now - self.t0

        if t < STARTUP_HOLD:
            self.publish_neutral()
            return

        phase = ((now - self.t0) / CYCLE_TIME) % 1.0
        
        if phase < self.prev_phase:
            self.cycles += 1
            self.get_logger().info(f"Step {self.cycles} completed")

        self.prev_phase = phase

        if self.cycles >= self.max_cycles:
            self.publish_neutral()
            self.get_logger().info("Forward step complete")
            self.timer.cancel()
            self.get_logger().info("Motion finished")
            return

        if phase < 0.5:
            swing_legs = SWING_A
            s = phase * 2
        else:
            swing_legs = SWING_B
            s = (phase - 0.5) * 2

        cmd = [0.0] * 12

        for leg in LEGS:
            swing = leg in swing_legs
            x, z = foot_position(s, swing)

            thigh_ik, knee_ik = leg_ik(x, z)

            thigh = THIGH_SIGN[leg] * thigh_ik
            knee  = KNEE_SIGN[leg] * knee_ik
            hip=HIP_SIGN[leg] * 0.06
            idx = LEG_INDEX[leg]
            cmd[idx+0] = hip
            cmd[idx+1] = thigh
            cmd[idx+2] = knee

        msg = Float64MultiArray()
        msg.data = cmd
        self.pub.publish(msg)

    def publish_neutral(self):
        msg = Float64MultiArray()
        msg.data = [-0.4 ,0.0 ,0.0,
                    0.4,0.0 ,0.0,
                    0.4 , 0.0 , 0.0,
                    -0.4 ,0.0 ,0.0]
        self.pub.publish(msg)

# ============================================================
def main():
    #steps = int(input("Enter number of steps: "))
    steps =1000
    rclpy.init()
    node = HybridIKTrot(steps)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
