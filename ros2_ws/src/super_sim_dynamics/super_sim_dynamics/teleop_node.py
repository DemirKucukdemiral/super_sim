#!/usr/bin/env python3
# Super Sim Pure Manual Vehicle Keyboard Teleoperation Node.

import fcntl
import json
import math
import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_keyboard_node")

        # Publishers
        self.drive_pub = self.create_publisher(Float32MultiArray, "/my_car/drive_request", 10)
        self.terrain_pub = self.create_publisher(String, "/my_car/set_terrain_type", 10)

        # Telemetry Subscription
        self.create_subscription(String, "/my_car/vehicle_status", self._on_status, 10)

        # Vehicle Control Demand States
        self.throttle = 0.0      # [-1.0 .. 1.0] (negative = Reverse)
        self.brake = 0.0         # [0.0 .. 1.0]
        self.steer = 0.0         # [-max_steer .. max_steer] (positive = left)
        self.handbrake = False

        self.latest_status = {}
        self.last_hud_print = 0.0

        self.terrain_profiles = ["HILLS", "FLAT", "ROLLER_COASTER"]
        self.curr_terrain_idx = 0

    def _on_status(self, msg: String):
        try:
            self.latest_status = json.loads(msg.data)
        except Exception:
            pass

    def publish_drive(self):
        msg = Float32MultiArray()
        msg.data = [
            float(self.throttle),
            float(self.brake),
            float(self.steer),
            1.0 if self.handbrake else 0.0
        ]
        self.drive_pub.publish(msg)

    def print_hud(self):
        now = time.time()
        if now - self.last_hud_print < 0.05:
            return
        self.last_hud_print = now

        s = self.latest_status
        speed = s.get("speed_kmh", 0.0)
        rpm = s.get("engine_rpm", 0.0)
        gear = s.get("gear", 1)
        p_bar = s.get("brake_line_bar", 0.0)
        abs_act = " [ABS]" if s.get("abs_active", False) else ""
        hb_act = " [HANDBRAKE]" if self.handbrake else ""
        alt = s.get("altitude_z", 0.57)
        grade = s.get("grade_pct", 0.0)
        terr = s.get("terrain", "HILLS")

        if self.throttle > 0.01:
            mode_tag = f"[D{gear}]"
        elif self.throttle < -0.01:
            mode_tag = "[R] "
        elif self.brake > 0.01:
            mode_tag = "[BRK]"
        else:
            mode_tag = "[N] "

        steer_deg = math.degrees(self.steer)
        if steer_deg > 0.5:
            steer_arrow = "<-"
        elif steer_deg < -0.5:
            steer_arrow = "->"
        else:
            steer_arrow = "--"

        hud = (
            f"{mode_tag} Spd: {speed:5.1f} km/h | "
            f"Gas: {max(0.0, self.throttle)*100:3.0f}% | "
            f"Brk: {self.brake*100:3.0f}% ({p_bar:4.1f} bar){abs_act}{hb_act} | "
            f"Steer: {steer_arrow} {abs(steer_deg):4.1f} deg | "
            f"Elev: {alt:+4.1f}m ({grade:+4.1f}%) [{terr}] | RPM: {rpm:4.0f}   "
        )
        sys.stdout.write(chr(13) + hud)
        sys.stdout.flush()


def get_key_tokens(fd):
    rlist, _, _ = select.select([fd], [], [], 0.0)
    if not rlist:
        return []
    try:
        raw_bytes = os.read(fd, 64)
        if not raw_bytes:
            return []
        raw = raw_bytes.decode("utf-8", errors="ignore")
    except (BlockingIOError, OSError):
        return []

    tokens = []
    i = 0
    n = len(raw)
    while i < n:
        if raw[i] == "\x1b" and i + 2 < n and raw[i+1] == "[":
            arrow = raw[i+2]
            if arrow == "A":
                tokens.append("up")
            elif arrow == "B":
                tokens.append("down")
            elif arrow == "C":
                tokens.append("right")
            elif arrow == "D":
                tokens.append("left")
            i += 3
        else:
            tokens.append(raw[i].lower())
            i += 1
    return tokens


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    banner = (
        "\n========================================================================\n"
        "                    SUPER SIM MANUAL DRIVING MODE                       \n"
        "========================================================================\n"
        "  [W] or [Up]     : Hold to Drive (Accelerate forward, release -> coast)\n"
        "  [S] or [Down]   : Hold to Brake (when moving); Hold to Reverse (when stopped)\n"
        "  [A] or [Left]   : Hold to Steer Left (Release -> instant spring-back)\n"
        "  [D] or [Right]  : Hold to Steer Right (Release -> instant spring-back)\n"
        "  [Space]         : Hold for Handbrake / Power Drift\n"
        "  [C]             : Instant Center Steering (0 deg)\n"
        "  [0]             : Full Stop & Reset to Standstill\n"
        "  [M]             : Toggle Terrain Environment (Flat vs Hills)\n"
        "  [Ctrl+C] or [Q] : Exit\n"
        "========================================================================\n"
    )
    print(banner)

    fd = sys.stdin.fileno()
    is_tty = sys.stdin.isatty()
    orig_settings = None
    orig_flags = None
    if is_tty:
        orig_settings = termios.tcgetattr(fd)
        tty.setraw(fd)
        orig_flags = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, orig_flags | os.O_NONBLOCK)

    last_w_time = 0.0
    last_s_time = 0.0
    last_a_time = 0.0
    last_d_time = 0.0
    last_space_time = 0.0
    last_any_key_time = 0.0

    loop_hz = 50.0
    loop_dt = 1.0 / loop_hz
    last_loop_time = time.time()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)
            now = time.time()
            dt = max(0.005, min(0.05, now - last_loop_time))
            last_loop_time = now

            tokens = get_key_tokens(fd)
            if tokens:
                last_any_key_time = now

            for token in tokens:
                if token in ("\x03", "q"):
                    return
                elif token in ("w", "up"):
                    last_w_time = now
                    last_s_time = 0.0
                elif token in ("s", "down"):
                    last_s_time = now
                    last_w_time = 0.0
                elif token in ("a", "left"):
                    last_a_time = now
                elif token in ("d", "right"):
                    last_d_time = now
                elif token == " ":
                    last_space_time = now
                elif token == "c":
                    node.steer = 0.0
                    last_a_time = 0.0
                    last_d_time = 0.0
                elif token == "0":
                    node.throttle = 0.0
                    node.brake = 0.0
                    node.steer = 0.0
                    node.handbrake = False
                    last_w_time = 0.0
                    last_s_time = 0.0
                    last_a_time = 0.0
                    last_d_time = 0.0
                    last_space_time = 0.0
                    sys.stdout.write(chr(10) + ">>> Reset to Standstill" + chr(10))
                elif token == "m":
                    node.curr_terrain_idx = (node.curr_terrain_idx + 1) % len(node.terrain_profiles)
                    new_terr = node.terrain_profiles[node.curr_terrain_idx]
                    node.terrain_pub.publish(String(data=new_terr))
                    sys.stdout.write(chr(10) + f">>> Switched Terrain Environment to: {new_terr}" + chr(10))

            # --- Multi-Key Hold Detection ---
            a_held = (now - last_a_time) < 0.28
            d_held = (now - last_d_time) < 0.28
            steering_active = a_held or d_held

            w_timeout = 0.55 if steering_active else 0.28
            s_timeout = 0.55 if steering_active else 0.28

            w_held = (now - last_w_time) < w_timeout and (now - last_any_key_time) < 0.35
            s_held = (now - last_s_time) < s_timeout and (now - last_any_key_time) < 0.35
            space_held = (now - last_space_time) < 0.28

            node.handbrake = space_held

            # --- Steering Dynamics ---
            curr_speed_kmh = node.latest_status.get("speed_kmh", 0.0)
            curr_speed_ms = abs(curr_speed_kmh) / 3.6
            max_steer = 0.65 / (1.0 + 0.004 * (curr_speed_ms ** 2))
            steer_rate = 5.2

            if a_held and not d_held:
                node.steer = min(max_steer, node.steer + steer_rate * dt)
            elif d_held and not a_held:
                node.steer = max(-max_steer, node.steer - steer_rate * dt)
            else:
                if abs(node.steer) > 0.015:
                    snap_speed = 8.5 * dt
                    if node.steer > 0:
                        node.steer = max(0.0, node.steer - snap_speed)
                    else:
                        node.steer = min(0.0, node.steer + snap_speed)
                else:
                    node.steer = 0.0

            # --- Longitudinal (Throttle / Brake / Reverse) Dynamics ---
            accel_rate = 12.0
            brake_rate = 14.0
            decay_rate = 9.0

            if w_held and not s_held:
                node.brake = 0.0
                if node.throttle < 0.0:
                    node.throttle = 0.0
                node.throttle = min(1.0, node.throttle + accel_rate * dt)

            elif s_held and not w_held:
                if curr_speed_kmh > 1.5:
                    node.throttle = 0.0
                    node.brake = min(1.0, node.brake + brake_rate * dt)
                else:
                    node.brake = 0.0
                    node.throttle = max(-1.0, node.throttle - accel_rate * dt)

            else:
                if node.throttle > 0.0:
                    node.throttle = max(0.0, node.throttle - decay_rate * dt)
                elif node.throttle < 0.0:
                    node.throttle = min(0.0, node.throttle + decay_rate * dt)

                if node.brake > 0.0:
                    node.brake = max(0.0, node.brake - decay_rate * dt)

            node.publish_drive()
            node.print_hud()
            time.sleep(loop_dt)

    except Exception as e:
        sys.stdout.write(chr(10) + f"Teleop Exception: {e}" + chr(10))
    finally:
        if orig_flags is not None:
            fcntl.fcntl(fd, fcntl.F_SETFL, orig_flags)
        if orig_settings is not None:
            termios.tcsetattr(fd, termios.TCSADRAIN, orig_settings)
        node.throttle = 0.0
        node.brake = 0.0
        node.steer = 0.0
        node.handbrake = False
        node.publish_drive()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("\nTeleop closed cleanly.")


if __name__ == "__main__":
    main()
