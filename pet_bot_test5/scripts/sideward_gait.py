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
CYCLE_TIME = 1.4#0.8

STEP_HEIGHT = 0.03 # 0.025
BODY_HEIGHT = 0.17 #0.17
SIDE_STEP = 0.03# 0.03

STARTUP_HOLD = 1.0

# +1 = move LEFT
# -1 = move RIGHT
DIRECTION=1

LEGS = ["FL", "FR", "RL", "RR"]
LEG_INDEX = {"FL": 0, "FR": 3, "RL": 6, "RR": 9}

SWING_A = ["FL", "RL"]
SWING_B = ["FR", "RR"]

# ============================================================
# JOINT SIGN CORRECTIONS (VERY IMPORTANT)
# ============================================================
HIP_SIGN = {"FL": -1, "FR": +1, "RL": +1, "RR": -1}
THIGH_SIGN = {"FL": +1, "FR": -1, "RL": -1, "RR": -1}
KNEE_SIGN  = {"FL": +1, "FR": -1, "RL": +1, "RR": -1}

# ============================================================
# 2D IK (UNCHANGED)
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
# 3D IK  (NEW)
# ============================================================
def leg_ik_3d(x, y, z):
    # Hip abduction/adduction
    hip = math.atan2(y, x)

    # project into sagittal plane
    xp = math.sqrt(x*x + y*y)

    # normal IK
    thigh, knee = leg_ik(xp, z)

    return hip, thigh, knee

# ============================================================
# SIDEWAYS FOOT TRAJECTORY
# ============================================================
def foot_position(s, swing,leg,DIRECTION):
    if swing:

        if DIRECTION == 1:

            if leg in SWING_A:
                y = -SIDE_STEP/4 + SIDE_STEP*s

            elif leg in SWING_B:
                y = -(-SIDE_STEP/2 + SIDE_STEP*s)

        elif DIRECTION == -1:

            if leg in SWING_A:
                y = -(-SIDE_STEP/2 + SIDE_STEP*s)

            elif leg in SWING_B:
                y = (-SIDE_STEP/4 + SIDE_STEP*s)

        z = BODY_HEIGHT - STEP_HEIGHT * math.sin(math.pi*s)
   

    else:
        # stance phase (push body)
        y = SIDE_STEP/2 - SIDE_STEP * s
        z = BODY_HEIGHT

    x = 0.02#0.02
    y = y

    return x, y, z
# ============================================================
# ROS NODE
# ============================================================
class SideWalk(Node):

    def __init__(self,steps):
        super().__init__("side_walk_controller")

        self.pub = self.create_publisher(
            Float64MultiArray,
            "/forward_position_controller/commands",
            10
        )

        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.timer = self.create_timer(1.0 / RATE, self.update)

        self.stance_x = {leg: 0.0 for leg in LEGS}
        self.stance_z = {leg: BODY_HEIGHT for leg in LEGS}
        self.stance_y = {leg: 0.0 for leg in LEGS}
        self.prev_swing = {leg: False for leg in LEGS}


        self.get_logger().info("Sideways walking started")

        self.prev_phase = 0.0
        self.cycles = 0
        self.max_cycles = steps

    # --------------------------------------------------------

    def update(self):

        now = self.get_clock().now().nanoseconds * 1e-9
        t = now - self.t0

        # startup stand
        if t < STARTUP_HOLD:
            self.publish_neutral()
            return

        phase = (t / CYCLE_TIME) % 1.0

        if phase < self.prev_phase:
            self.cycles += 1

        self.prev_phase = phase

        if self.cycles >= self.max_cycles:
            self.publish_neutral()
            self.get_logger().info("Sideways step complete")
            self.timer.cancel()
            return

        # Phase 1: left step
        if phase < 0.25:
            swing_legs = SWING_A
            s = phase * 4

        # Phase 2: pause
        elif phase < 0.5:
            swing_legs = []
            s = 0

        # Phase 3: right step
        elif phase < 0.75:
            swing_legs = SWING_B
            s = (phase - 0.5) * 4

        # Phase 4: pause
        else:
            swing_legs = []
            s = 0

        cmd = [0.0] * 12

        # ------------ MAIN IK LOOP -------------
        for leg in LEGS:

            swing = leg in swing_legs

            x, y, z = foot_position(s, swing,leg,DIRECTION)

            if not swing:
                if self.prev_swing[leg]:
                    self.stance_x[leg] = x
                    self.stance_z[leg] = z  
                    self.stance_y[leg] = y 
                    
                x = self.stance_x[leg]
                z = self.stance_z[leg]
                y = self.stance_y[leg]


            # if leg in ["FR", "RR"]:
            #     y = DIRECTION*y

            hip_ik, thigh_ik, knee_ik = leg_ik_3d(x, y, z)

            hip   = HIP_SIGN[leg] * hip_ik
            thigh = THIGH_SIGN[leg] * thigh_ik
            knee  = KNEE_SIGN[leg]  * knee_ik

            idx = LEG_INDEX[leg]

            cmd[idx+0] = hip
            cmd[idx+1] = thigh
            cmd[idx+2] = knee

            self.prev_swing[leg] = swing

        msg = Float64MultiArray()
        msg.data = cmd
        self.pub.publish(msg)

    # --------------------------------------------------------

    def publish_neutral(self):
        msg = Float64MultiArray()
        msg.data = [
            -0.4, 0.0, 0.0,
             0.4, 0.0, 0.0,
             0.4, 0.0, 0.0,
            -0.4, 0.0, 0.0
        ]
        self.pub.publish(msg)

# ============================================================
def main():
    steps = int(input("Enter number of sideways steps: "))+1
    direction=int(input("Enter direction \n" \
                        "Left=1\n" \
                        "Right=-1\n" \
                        ": "))
    
    global DIRECTION
    DIRECTION=direction
    rclpy.init()
    node = SideWalk(steps)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()