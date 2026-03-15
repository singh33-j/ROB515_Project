#!/usr/bin/python3
# coding=utf8
"""
Sub arm (PatsyCline) - Dual-arm block transfer
Waits for host signal, moves to handoff position with gripper rotated 90deg,
grabs block from top/bottom, then places it at origin.
"""
import sys
sys.path.append('/home/pi/ArmPi/')
import time
import math
import threading
import json
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from http.server import HTTPServer, BaseHTTPRequestHandler
import requests

# ---- Configuration ----
HOST_PI = 'MartyRobbins.engr.oregonstate.edu'
SIGNAL_PORT = 9090

# Handoff position in SUB's coordinate frame
# Host holds at (0, 10, 10) in its frame
# Mirrored: sub_x = -host_x = 0, sub_y = 2 * host_y_from_crosshair, sub_z = same
# The host is at y=10 from its base, crosshair is at y~20 from base (image_center_distance)
# so host_y_from_crosshair = 10 - 20 = -10 (block is between base and crosshair)
# Actually: the coordinate system origin is at the arm base, y increases away from arm.
# Crosshair is ~20cm from base. Host holds at y=10, so it's 10cm from its base.
# For the sub (on opposite side), the same physical point is further away.
# We need to calibrate this - starting with the same coordinates and adjusting.
HANDOFF_COORD = (0, 20, 20)  # (x, y, z) cm - handoff position 20cm above crosshair

PLACE_COORD = (0, 20, 1.5)  # Where to place the block (on the mat at crosshair)

# ---- Arm setup ----
AK = ArmIK()
servo1 = 500  # gripper closed position

# ---- State machine ----
class TransferState:
    WAITING = 0
    APPROACHING = 1
    GRABBING = 2
    WAITING_RELEASE = 3
    PLACING = 4
    RETURNING = 5
    DONE = 6

state = TransferState.WAITING
state_lock = threading.Lock()
host_ready = threading.Event()
host_released = threading.Event()

# ---- Signal server (receives signals from host) ----
class SignalHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        length = int(self.headers.get('Content-Length', 0))
        body = json.loads(self.rfile.read(length)) if length else {}
        signal = body.get('signal', '')

        if signal == 'ready_to_grab':
            print("[SUB] Host is ready for handoff")
            host_ready.set()
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b'{"ok": true}')
        elif signal == 'released':
            print("[SUB] Host has released the block")
            host_released.set()
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b'{"ok": true}')
        else:
            self.send_response(400)
            self.end_headers()
            self.wfile.write(b'{"ok": false}')

    def log_message(self, format, *args):
        pass

def start_signal_server():
    server = HTTPServer(('0.0.0.0', SIGNAL_PORT), SignalHandler)
    server.serve_forever()

def send_signal_to_host(signal):
    """Send a coordination signal to the host Pi."""
    url = f'http://{HOST_PI}:{SIGNAL_PORT}'
    try:
        r = requests.post(url, json={'signal': signal}, timeout=5)
        return r.status_code == 200
    except Exception as e:
        print(f"[SUB] Failed to signal host: {e}")
        return False

# ---- Init ----
def init_move():
    Board.setBusServoPulse(1, servo1 - 50, 300)
    Board.setBusServoPulse(2, 500, 500)
    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

# ---- Main logic ----
def main():
    global state

    # Start signal server thread
    sig_thread = threading.Thread(target=start_signal_server, daemon=True)
    sig_thread.start()
    print("[SUB] Signal server listening on port", SIGNAL_PORT)

    # Init arm
    init_move()
    time.sleep(1.5)
    print("[SUB] Ready. Waiting for host signal...")

    try:
        while True:
            if state == TransferState.WAITING:
                # Wait for host to signal it's holding the block
                host_ready.wait(timeout=1)
                if host_ready.is_set():
                    host_ready.clear()
                    print("[SUB] Moving to handoff position...")
                    state = TransferState.APPROACHING

            elif state == TransferState.APPROACHING:
                # Open gripper wide
                Board.setBusServoPulse(1, servo1 - 280, 500)
                time.sleep(0.5)

                # Rotate gripper 90 degrees to grab top/bottom
                # Servo 2 at 500 = default, 500 +/- ~250 = 90 deg rotation
                # We want perpendicular to host's grip
                Board.setBusServoPulse(2, 162, 500)  # ~120deg rotated from default
                time.sleep(0.5)

                # Move to handoff position
                result = AK.setPitchRangeMoving(HANDOFF_COORD, -90, -90, 0)
                if result:
                    time.sleep(result[2] / 1000 + 0.5)
                    print("[SUB] At handoff position. Grabbing...")
                    state = TransferState.GRABBING
                else:
                    print("[SUB] WARNING: Handoff position unreachable!")
                    # Try adjusting
                    result = AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90)
                    if result:
                        time.sleep(result[2] / 1000 + 0.5)
                    state = TransferState.GRABBING

            elif state == TransferState.GRABBING:
                # Close gripper on block (top/bottom grip)
                Board.setBusServoPulse(1, servo1, 500)
                time.sleep(1)

                # Signal host that we've grabbed
                print("[SUB] Grabbed block. Signaling host...")
                send_signal_to_host('grab_confirmed')
                state = TransferState.WAITING_RELEASE

            elif state == TransferState.WAITING_RELEASE:
                # Wait for host to release
                host_released.wait(timeout=10)
                if host_released.is_set():
                    host_released.clear()
                    print("[SUB] Host released. Moving to place position...")
                    state = TransferState.PLACING
                else:
                    print("[SUB] Timeout waiting for host release. Retrying...")

            elif state == TransferState.PLACING:
                # Lift slightly first
                AK.setPitchRangeMoving((HANDOFF_COORD[0], HANDOFF_COORD[1], HANDOFF_COORD[2] + 3),
                                       -90, -90, 0, 800)
                time.sleep(1)

                # Reset gripper rotation for placing
                Board.setBusServoPulse(2, 500, 500)
                time.sleep(0.5)

                # Move above place position
                result = AK.setPitchRangeMoving((PLACE_COORD[0], PLACE_COORD[1], 12),
                                                -90, -90, 0)
                if result:
                    time.sleep(result[2] / 1000 + 0.5)

                # Lower to place position
                AK.setPitchRangeMoving(PLACE_COORD, -90, -90, 0, 1000)
                time.sleep(1.5)

                # Open gripper to release
                Board.setBusServoPulse(1, servo1 - 200, 500)
                time.sleep(0.8)

                # Lift away
                AK.setPitchRangeMoving((PLACE_COORD[0], PLACE_COORD[1], 12),
                                       -90, -90, 0, 800)
                time.sleep(1)

                print("[SUB] Block placed! Returning to home...")
                state = TransferState.RETURNING

            elif state == TransferState.RETURNING:
                init_move()
                time.sleep(1.5)
                print("[SUB] Transfer complete!")
                state = TransferState.DONE

            elif state == TransferState.DONE:
                time.sleep(1)
                break

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[SUB] Interrupted")
    finally:
        init_move()
        time.sleep(1)

if __name__ == '__main__':
    main()
