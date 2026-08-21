#!/usr/bin/env python3

import os
import signal
import subprocess
import sys
import termios
import tty
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class KeyboardTeleop(Node):

    def __init__(self):

        super().__init__("keyboard_teleop")

        self.publisher = self.create_publisher(
            Float64MultiArray,
            "/forward_position_controller/commands",
            10
        )


        self.active_process = None

        self.current_gait = None

        print("\n========================================")
        print("╠══════════════════════════════════════╣")
        print("║                                      ║")
        print("║               ┌─────┐                ║")
        print("║               │  W  │                ║")
        print("║               │  ↑  │                ║")
        print("║               └─────┘                ║")
        print("║          ┌─────┬─────┬─────┐         ║")
        print("║          │  A  │  S  │  D  │         ║")
        print("║          │  ←  │  ↓  │  →  │         ║")
        print("║          └─────┴─────┴─────┘         ║")
        print("║                                      ║")
        print("║          ┌─────┐     ┌─────┐         ║")
        print("║          │  Z  │     │  X  │         ║")
        print("║          │  ↶  │     │  ↷  │         ║")
        print("║          └─────┘     └─────┘         ║")
        print("║        ROTATE LEFT   ROTATE RIGHT    ║")
        print("║                                      ║")
        print("║             ┌───────────┐            ║")
        print("║             │   SPACE   │            ║")
        print("║             │   STOP    │            ║")
        print("║             └───────────┘            ║")
        print("║                                      ║")
        print("║               [ Q ] QUIT             ║")
        print("║                                      ║")
        print("╚══════════════════════════════════════╝")
        print("========================================\n")

    def publish_neutral_pose(self):

        msg = Float64MultiArray()

        msg.data = [
            -0.4, 0.0, 0.0,
             0.4, 0.0, 0.0,
             0.4, 0.0, 0.0,
            -0.4, 0.0, 0.0
        ]

        self.publisher.publish(msg)

    

    def stop_active_gait(self):

        if self.active_process is not None:

            process = self.active_process

            if process.poll() is None:

                self.get_logger().info(
                    "Stopping current gait..."
                )

                try:
                    
                    os.killpg(
                        os.getpgid(process.pid),
                        signal.SIGTERM
                    )

                    process.wait(timeout=2)

                except subprocess.TimeoutExpired:

                    self.get_logger().warning(
                        "Gait did not stop. Force killing..."
                    )

                    try:
                        os.killpg(
                            os.getpgid(process.pid),
                            signal.SIGKILL
                        )
                    except ProcessLookupError:
                        pass

                    process.wait()

                except ProcessLookupError:
                    pass

       
        self.active_process = None
        self.current_gait = None


        self.get_logger().info(
            "Moving robot to neutral pose..."
        )

        self.publish_neutral_pose()

        
        time.sleep(0.5)

    def start_gait(
        self,
        script_name,
        gait_name,
        script_input=None
    ):

        if self.current_gait == gait_name:
            return

     
        self.stop_active_gait()

       
        self.get_logger().info(
            f"Starting {gait_name} gait"
        )

        if script_input is not None:

            self.active_process = subprocess.Popen(
                [
                    "ros2",
                    "run",
                    "pet_bot_test5",
                    script_name
                ],

                stdin=subprocess.PIPE,

                text=True,

                start_new_session=True
            )

            
            self.active_process.stdin.write(
                script_input
            )

            self.active_process.stdin.flush()

        else:

            self.active_process = subprocess.Popen(
                [
                    "ros2",
                    "run",
                    "pet_bot_test5",
                    script_name
                ],

                start_new_session=True
            )

        self.current_gait = gait_name

   
    def get_key(self):

        terminal_settings = termios.tcgetattr(
            sys.stdin
        )

        try:

            tty.setraw(
                sys.stdin.fileno()
            )

            key = sys.stdin.read(1)

        finally:

            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                terminal_settings
            )

        return key.lower()

    def run(self):

        try:

            while rclpy.ok():

                key = self.get_key()

                if key == "w":

                    self.start_gait(
                        "forward_gait.py",
                        "FORWARD"
                    )

                elif key == "s":

                    self.start_gait(
                        "backward_gait.py",
                        "BACKWARD"
                    )
             
                elif key == "a":

                    self.start_gait(
                        "sideward_gait.py",
                        "LEFT",
                        "1\n"
                    )
               
                elif key == "d":

                    self.start_gait(
                        "sideward_gait.py",
                        "RIGHT",
                        "-1\n"
                    )

                elif key == "z":

                    self.start_gait(
                        "rotate_gait.py",
                        "LEFT",
                        "1\n"
                    )
               
                elif key == "c":

                    self.start_gait(
                        "rotate_gait.py",
                        "RIGHT",
                        "-1\n"
                    )    

                elif key == "x":

                    self.stop_active_gait()

                    print("\nStopped.")

               
                elif key == "q":

                    self.stop_active_gait()

                    print("\nTeleop closed.")

                    break

        except KeyboardInterrupt:

            self.stop_active_gait()

    
    def shutdown(self):

        self.stop_active_gait()


def main():

    rclpy.init()

    node = KeyboardTeleop()

    try:

        node.run()

    finally:

        node.shutdown()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
