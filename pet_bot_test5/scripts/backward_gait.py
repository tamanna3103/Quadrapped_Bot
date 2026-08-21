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
# GAIT PARAMETERS
# ============================================================
RATE = 50.0
CYCLE_TIME = 1.0
STEP_LENGTH = 0.035
STEP_HEIGHT = 0.02
BODY_HEIGHT = 0.12
DIRECTION = -1          # backward
STARTUP_HOLD = 1.0

# ============================================================
# LEGS
# ============================================================
LEGS = ["FL", "FR", "RL", "RR"]
LEG_INDEX = {"FL": 0, "FR": 3, "RL": 6, "RR": 9}

SWING_A = ["FL", "RR"]
SWING_B = ["FR", "RL"]

HIP_SIGN   = {"FL": -1, "FR": +1, "RL": +1, "RR": -1}
THIGH_SIGN = {"FL": +1, "FR": -1, "RL": -1, "RR": -1}
KNEE_SIGN  = {"FL": +1, "FR": -1, "RL": +1, "RR": -1}

# ============================================================
# IK
# ============================================================
def leg_ik(x, z):
    r = math.hypot(x, z)
    r = max(min(r, L1 + L2 - 1e-6), abs(L1 - L2) + 1e-6)

    cos_k = (L1**2 + L2**2 - r**2) / (2 * L1 * L2)
    knee = -(math.pi - math.acos(cos_k))

    phi = math.atan2(z, x)
    cos_a = (L1**2 + r**2 - L2**2) / (2 * L1 * r)
    alpha = math.acos(cos_a)

    thigh = phi + alpha - math.pi / 2
    return thigh, knee

# ============================================================
# FOOT TRAJECTORY (BODY FRAME)
# ============================================================
def foot_position(s, swing):
    if swing:
        x = DIRECTION * (-STEP_LENGTH / 2 + STEP_LENGTH * s)
        z = BODY_HEIGHT - STEP_HEIGHT * (math.sin(math.pi * s)**2)
    else:
        x = DIRECTION * (STEP_LENGTH / 2)
        z = BODY_HEIGHT
    return x, z

# ============================================================
# NODE
# ============================================================
class HybridIKTrot_Backward(Node):

    def __init__(self,steps):
        super().__init__("hybrid_ik_trot_backward")

        self.pub = self.create_publisher(
            Float64MultiArray,
            "/forward_position_controller/commands",
            10
        )

        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.timer = self.create_timer(1.0 / RATE, self.update)

        # world-frame stance memory
        self.stance_x = {leg: 0.0 for leg in LEGS}
        self.stance_z = {leg: BODY_HEIGHT for leg in LEGS}
        self.prev_swing = {leg: False for leg in LEGS}

        self.prev_phase = 0.0
        self.cycles = 0
        self.max_cycles = steps

        self.returning_neutral = False
        self.start_pose_done = False

        self.get_logger().info("Backward motion started")
    # ========================================================
    def update(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        t = now - self.t0

        # move robot to start pose before trot
        if not self.start_pose_done:
            if t < STARTUP_HOLD:
                self.publish_start_pose()
                return
            else:
                self.start_pose_done = True
                self.t0 = now   # reset gait timer
                return

        # hold neutral before shutting down
        if self.returning_neutral:
            if now - self.neutral_start > 1.5:
                self.timer.cancel()
                self.get_logger().info("Motion finished")
            else:
                self.publish_neutral()
            return

        phase = ((now - self.t0) / CYCLE_TIME) % 1.0

        if phase < self.prev_phase:
            self.cycles += 1
            self.get_logger().info(f"Step {self.cycles} completed")

        self.prev_phase = phase

        if self.cycles >= self.max_cycles and not self.returning_neutral:
            self.publish_neutral()
            self.get_logger().info("Returning to neutral position")

            self.returning_neutral = True
            self.neutral_start = now
            return

        if phase < 0.5:
            swing_legs = SWING_A
            s = phase * 2.0
        else:
            swing_legs = SWING_B
            s = (phase - 0.5) * 2.0

        stance_diagonal = SWING_B if swing_legs == SWING_A else SWING_A
        roll_mag = 0.06 * math.sin(math.pi * s)

        cmd = [0.0] * 12

        for leg in LEGS:
            swing = leg in swing_legs
            x, z = foot_position(s, swing)

            # -------- WORLD FRAME STANCE LOCK (X + Z) --------
            if not swing:
                if self.prev_swing[leg]:
                    self.stance_x[leg] = x
                    self.stance_z[leg] = z
                x = self.stance_x[leg]
                z = self.stance_z[leg]

            # rear-leg load bias
            if leg in ["RL", "RR"]:
                z += 0.008

            thigh_ik, knee_ik = leg_ik(x, z)

            thigh = THIGH_SIGN[leg] * (thigh_ik - 0.4
                                       )
            knee  = KNEE_SIGN[leg]  * (knee_ik + 0.35)

            # -------- BODY-LEVEL ROLL STABILIZATION --------
            if leg in stance_diagonal:
                hip = HIP_SIGN[leg] * (0.12 - roll_mag)
            else:
                hip = HIP_SIGN[leg] * (0.12 + roll_mag)

            idx = LEG_INDEX[leg]
            cmd[idx + 0] = hip
            cmd[idx + 1] = thigh
            cmd[idx + 2] = knee

            self.prev_swing[leg] = swing

        msg = Float64MultiArray()
        msg.data = cmd
        self.pub.publish(msg)

    # ========================================================
    def publish_neutral(self):
        msg = Float64MultiArray()
        msg.data = [
            -0.0, 0.0, -0.0,
             0.0, 0.0, -0.0,
             0.0, 0.0, -0.0,
            -0.0, 0.0, -0.0
        ]
        self.pub.publish(msg)

    def publish_start_pose(self):
        msg = Float64MultiArray()
        msg.data = [
            -0.4, 0.0, 0.6,
            0.4, 0.0, -0.6,
            0.4, 0.0, 0.6,
            -0.4, 0.0, -0.6
        ]
        self.pub.publish(msg)

# ============================================================
def main():
    #steps = int(input("Enter number of steps: "))
    steps =1000
    rclpy.init()
    node = HybridIKTrot_Backward(steps)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
