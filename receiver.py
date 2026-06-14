#!/usr/bin/env python3
"""
serial_bridge.py
----------------
Serial communication bridge between a host PC and the Arduino omni-base.

Protocol (matches the Arduino firmware):
  Host  → Arduino : "linear_x,linear_y,angular_z\n"   (floats, e.g. "0.5,0.0,0.1\n")
  Arduino → Host  : "vx,vy,wz\n"                      (odometry CSV at ~20 Hz)
  Arduino → Host  : "[DBG] ...\n"                      (debug lines, filtered out)

Usage examples
--------------
1. Interactive keyboard teleop:
       python3 serial_bridge.py --port /dev/ttyUSB0 --teleop

2. ROS bridge (publishes /odom, subscribes /cmd_vel):
       python3 serial_bridge.py --port /dev/ttyUSB0 --ros

3. Headless (just print odometry to terminal):
       python3 serial_bridge.py --port /dev/ttyUSB0
"""

import argparse
import sys
import threading
import time
import serial


# ─────────────────────────────────────────────────────────────────────────────
#  Serial Bridge Core
# ─────────────────────────────────────────────────────────────────────────────

class SerialBridge:
    """
    Manages bidirectional serial communication with the Arduino firmware.

    Public API:
        send_cmd_vel(linear_x, linear_y, angular_z)  – non-blocking send
        get_odometry()                                – returns latest (vx, vy, wz)
        start() / stop()
    """

    def __init__(self, port: str, baudrate: int = 115200, cmd_rate_hz: int = 20):
        self._port      = port
        self._baudrate  = baudrate
        self._cmd_rate  = cmd_rate_hz          # must match Arduino COMMAND_RATE

        # Latest command (written by main thread, read by TX thread)
        self._cmd_lock   = threading.Lock()
        self._linear_x   = 0.0
        self._linear_y   = 0.0
        self._angular_z  = 0.0

        # Latest odometry (written by RX thread, read by main thread)
        self._odom_lock  = threading.Lock()
        self._odom       = (0.0, 0.0, 0.0)    # (vx, vy, wz)

        self._ser        = None
        self._running    = False
        self._tx_thread  = None
        self._rx_thread  = None

    # ── Public ───────────────────────────────────────────────────────────────

    def start(self):
        """Open the serial port and launch background threads."""
        self._ser = serial.Serial(
            port      = self._port,
            baudrate  = self._baudrate,
            timeout   = 1.0,
        )
        time.sleep(2.0)          # wait for Arduino reset after DTR toggle
        self._ser.reset_input_buffer()

        self._running = True
        self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._tx_thread.start()
        self._rx_thread.start()
        print(f"[bridge] connected on {self._port} @ {self._baudrate} baud")

    def stop(self):
        """Send a stop command, then close the port."""
        self.send_cmd_vel(0.0, 0.0, 0.0)
        time.sleep(0.1)
        self._running = False
        if self._ser and self._ser.is_open:
            self._ser.close()
        print("[bridge] disconnected")

    def send_cmd_vel(self, linear_x: float, linear_y: float, angular_z: float):
        """Thread-safe update of the velocity setpoint."""
        with self._cmd_lock:
            self._linear_x  = float(linear_x)
            self._linear_y  = float(linear_y)
            self._angular_z = float(angular_z)

    def get_odometry(self) -> tuple:
        """Returns the latest (vx, vy, wz) tuple received from the Arduino."""
        with self._odom_lock:
            return self._odom

    # ── Background threads ───────────────────────────────────────────────────

    def _tx_loop(self):
        """Sends cmd_vel to Arduino at cmd_rate_hz."""
        period = 1.0 / self._cmd_rate
        while self._running:
            t0 = time.monotonic()
            with self._cmd_lock:
                msg = f"{self._linear_x:.4f},{self._linear_y:.4f},{self._angular_z:.4f}\n"
            try:
                self._ser.write(msg.encode())
            except serial.SerialException as e:
                print(f"[bridge][TX] error: {e}")
                self._running = False
                break
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, period - elapsed))

    def _rx_loop(self):
        """Reads odometry lines from Arduino and updates self._odom."""
        while self._running:
            try:
                raw = self._ser.readline()
                if not raw:
                    continue
                line = raw.decode(errors="ignore").strip()

                # Skip debug lines
                if line.startswith("[DBG]"):
                    print(f"[arduino] {line}")
                    continue

                parts = line.split(",")
                if len(parts) == 3:
                    vx, vy, wz = float(parts[0]), float(parts[1]), float(parts[2])
                    with self._odom_lock:
                        self._odom = (vx, vy, wz)

            except (ValueError, UnicodeDecodeError):
                pass   # malformed line – skip
            except serial.SerialException as e:
                print(f"[bridge][RX] error: {e}")
                self._running = False
                break


# ─────────────────────────────────────────────────────────────────────────────
#  Keyboard Teleop
# ─────────────────────────────────────────────────────────────────────────────

TELEOP_HELP = """
╔══════════════════════════════════════╗
║       Keyboard Teleop Controls       ║
╠══════════════════════════════════════╣
║  W / S  – forward / backward (vx)   ║
║  A / D  – strafe left / right (vy)  ║
║  Q / E  – rotate CCW / CW (wz)      ║
║  SPACE  – stop all                   ║
║  +/-    – increase/decrease speed    ║
║  X      – quit                       ║
╚══════════════════════════════════════╝
"""

def run_teleop(bridge: SerialBridge):
    """Non-blocking keyboard teleop using tty raw mode."""
    import tty, termios

    print(TELEOP_HELP)

    step     = 0.05   # velocity step per keypress (m/s or rad/s)
    vx = vy = wz = 0.0

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while True:
            ch = sys.stdin.read(1).lower()

            if ch == 'x':
                break
            elif ch == 'w':
                vx = min(vx + step, 1.0)
            elif ch == 's':
                vx = max(vx - step, -1.0)
            elif ch == 'a':
                vy = min(vy + step, 1.0)
            elif ch == 'd':
                vy = max(vy - step, -1.0)
            elif ch == 'q':
                wz = min(wz + step, 2.0)
            elif ch == 'e':
                wz = max(wz - step, -2.0)
            elif ch == ' ':
                vx = vy = wz = 0.0
            elif ch == '+':
                step = min(step + 0.01, 0.5)
            elif ch == '-':
                step = max(step - 0.01, 0.01)

            bridge.send_cmd_vel(vx, vy, wz)
            odom = bridge.get_odometry()
            # Clear line and print status
            sys.stdout.write(
                f"\r cmd: vx={vx:+.2f}  vy={vy:+.2f}  wz={wz:+.2f}  |  "
                f"odom: vx={odom[0]:+.3f}  vy={odom[1]:+.3f}  wz={odom[2]:+.3f}  "
                f"step={step:.2f}   "
            )
            sys.stdout.flush()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print()


# ─────────────────────────────────────────────────────────────────────────────
#  ROS Bridge
# ─────────────────────────────────────────────────────────────────────────────

def run_ros_bridge(bridge: SerialBridge):
    """
    Bridges to ROS:
      Subscribes : /cmd_vel  (geometry_msgs/Twist)
      Publishes  : /odom     (nav_msgs/Odometry)  – velocity fields only
    """
    try:
        import rospy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
    except ImportError:
        print("[ros] rospy not found – install ROS and source your setup.bash")
        return

    rospy.init_node("serial_bridge", anonymous=False)

    def cmd_vel_cb(msg: Twist):
        bridge.send_cmd_vel(msg.linear.x, msg.linear.y, msg.angular.z)

    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_cb, queue_size=1)
    odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)

    rate = rospy.Rate(20)   # publish odom at 20 Hz
    print("[ros] node started – /cmd_vel → Arduino → /odom")

    while not rospy.is_shutdown():
        vx, vy, wz = bridge.get_odometry()

        odom = Odometry()
        odom.header.stamp    = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id  = "base_link"
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = vy
        odom.twist.twist.angular.z = wz
        odom_pub.publish(odom)

        rate.sleep()


# ─────────────────────────────────────────────────────────────────────────────
#  Headless monitor (default mode)
# ─────────────────────────────────────────────────────────────────────────────

def run_monitor(bridge: SerialBridge):
    """Just print odometry to stdout – useful for testing."""
    print("[monitor] printing odometry (Ctrl+C to quit)")
    try:
        while True:
            vx, vy, wz = bridge.get_odometry()
            print(f"  odom → vx={vx:+.4f}  vy={vy:+.4f}  wz={wz:+.4f}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass


# ─────────────────────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────────────────────

def list_ports():
    """Print available serial ports and exit."""
    try:
        from serial.tools import list_ports as lp
        ports = lp.comports()
        if not ports:
            print("No serial ports found.")
        else:
            print("Available serial ports:")
            for p in ports:
                print(f"  {p.device:20s}  {p.description}")
    except ImportError:
        print("Install pyserial: pip install pyserial")
    sys.exit(0)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Serial bridge for Arduino omni-base firmware",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--port",    default="/dev/ttyUSB0",
                        help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--baud",    type=int, default=115200,
                        help="Baud rate (default: 115200)")
    parser.add_argument("--teleop",  action="store_true",
                        help="Run keyboard teleop")
    parser.add_argument("--ros",     action="store_true",
                        help="Run ROS bridge (requires rospy)")
    parser.add_argument("--list",    action="store_true",
                        help="List available serial ports and exit")
    return parser.parse_args()


def main():
    args = parse_args()

    if args.list:
        list_ports()

    bridge = SerialBridge(port=args.port, baudrate=args.baud)

    try:
        bridge.start()

        if args.teleop:
            run_teleop(bridge)
        elif args.ros:
            run_ros_bridge(bridge)
        else:
            run_monitor(bridge)

    except serial.SerialException as e:
        print(f"[error] could not open {args.port}: {e}")
        print("        Run with --list to see available ports.")
        sys.exit(1)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()


if __name__ == "__main__":
    main()