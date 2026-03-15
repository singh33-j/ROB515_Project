#!/usr/bin/python3
# coding=utf8
"""
Sub arm (PatsyCline) - Dual-arm block transfer
Waits for host signal with block positions (already in sub coords),
grabs block at handoff with gripper rotated 90deg,
then stacks it on the blue block position.
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
STACK_HEIGHT = 1.5  # z for placing block on top of blue (cm above mat)

# ---- Arm setup ----
AK = ArmIK()
servo1 = 500

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
host_ready = threading.Event()
host_released = threading.Event()

# Positions received from host (already in sub's coordinate frame)
handoff_coord = None
blue_coord = None

# ---- Signal server (receives signals from host) ----
class SignalHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        global handoff_coord, blue_coord
        length = int(self.headers.get('Content-Length', 0))
        body = json.loads(self.rfile.read(length)) if length else {}
        signal = body.get('signal', '')
        data = body.get('data', {})

        if signal == 'ready_to_grab':
            # Extract positions (already transformed to sub coords by host)
            if 'handoff' in data:
                handoff_coord = tuple(data['handoff'])
                print(f"[SUB] Handoff position (sub coords): {handoff_coord}")
            if 'blue' in data:
                blue_coord = (data['blue'][0], data['blue'][1])
                print(f"[SUB] Blue block position (sub coords): {blue_coord}")
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

    sig_thread = threading.Thread(target=start_signal_server, daemon=True)
    sig_thread.start()
    print("[SUB] Signal server listening on port", SIGNAL_PORT)

    init_move()
    time.sleep(1.5)
    print("[SUB] Ready. Waiting for host signal...")

    try:
        while True:
            if state == TransferState.WAITING:
                host_ready.wait(timeout=1)
                if host_ready.is_set():
                    host_ready.clear()
                    if handoff_coord is None:
                        print("[SUB] ERROR: No handoff position received!")
                        continue
                    print(f"[SUB] Moving to handoff at {handoff_coord}...")
                    state = TransferState.APPROACHING

            elif state == TransferState.APPROACHING:
                # Open gripper wide
                Board.setBusServoPulse(1, servo1 - 280, 500)
                time.sleep(0.5)

                # Rotate gripper 90deg to grab top/bottom
                Board.setBusServoPulse(2, 162, 500)  # ~120deg rotated
                time.sleep(0.5)

                # Move to handoff position
                result = AK.setPitchRangeMoving(handoff_coord, -90, -90, 0)
                if result:
                    time.sleep(result[2] / 1000 + 0.5)
                    print("[SUB] At handoff position. Grabbing...")
                    state = TransferState.GRABBING
                else:
                    print("[SUB] WARNING: Handoff unreachable, trying wider pitch...")
                    result = AK.setPitchRangeMoving(handoff_coord, -30, -30, -90)
                    if result:
                        time.sleep(result[2] / 1000 + 0.5)
                    state = TransferState.GRABBING

            elif state == TransferState.GRABBING:
                # Close gripper (top/bottom grip)
                Board.setBusServoPulse(1, servo1, 500)
                time.sleep(1)

                print("[SUB] Grabbed block. Signaling host...")
                send_signal_to_host('grab_confirmed')
                state = TransferState.WAITING_RELEASE

            elif state == TransferState.WAITING_RELEASE:
                host_released.wait(timeout=10)
                if host_released.is_set():
                    host_released.clear()
                    print("[SUB] Host released. Placing on blue block...")
                    state = TransferState.PLACING
                else:
                    print("[SUB] Timeout waiting for host release. Retrying...")

            elif state == TransferState.PLACING:
                # Lift slightly
                AK.setPitchRangeMoving((handoff_coord[0], handoff_coord[1], handoff_coord[2] + 3),
                                       -90, -90, 0, 800)
                time.sleep(1)

                # Reset gripper rotation for placing
                Board.setBusServoPulse(2, 500, 500)
                time.sleep(0.5)

                if blue_coord:
                    place_x, place_y = blue_coord
                else:
                    # Fallback to crosshair
                    place_x, place_y = 0, 20

                # Move above blue block
                result = AK.setPitchRangeMoving((place_x, place_y, 12), -90, -90, 0)
                if result:
                    time.sleep(result[2] / 1000 + 0.5)

                # Lower to stack height (on top of blue block)
                AK.setPitchRangeMoving((place_x, place_y, STACK_HEIGHT + 3), -90, -90, 0, 500)
                time.sleep(0.5)

                AK.setPitchRangeMoving((place_x, place_y, STACK_HEIGHT), -90, -90, 0, 1000)
                time.sleep(1)

                # Open gripper to release
                Board.setBusServoPulse(1, servo1 - 200, 500)
                time.sleep(0.8)

                # Lift away
                AK.setPitchRangeMoving((place_x, place_y, 12), -90, -90, 0, 800)
                time.sleep(1)

                print("[SUB] Block placed on blue! Returning to home...")
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
