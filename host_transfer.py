#!/usr/bin/python3
# coding=utf8
"""
Host arm (MartyRobbins) - Dual-arm block transfer
Detects green block, picks it up, holds at handoff position,
waits for sub arm to grab, then releases.
"""
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import math
import threading
import numpy as np
import Camera
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *
from http.server import HTTPServer, BaseHTTPRequestHandler
import requests
import json

# ---- Configuration ----
SUB_PI_HOST = 'PatsyCline.engr.oregonstate.edu'
HANDOFF_COORD = (0, 20, 20)  # (x, y, z) cm - handoff position 20cm above crosshair
SIGNAL_PORT = 9090           # port for coordination signals on this Pi
TARGET_COLOR = 'green'

# ---- Arm setup ----
AK = ArmIK()
servo1 = 500  # gripper closed position

# ---- State machine ----
class TransferState:
    DETECTING = 0
    PICKING = 1
    MOVING_TO_HANDOFF = 2
    HOLDING = 3         # waiting for sub to grab
    RELEASING = 4
    RETURNING = 5
    DONE = 6

state = TransferState.DETECTING
state_lock = threading.Lock()
sub_grabbed = threading.Event()

# ---- Signal server (receives signals from sub) ----
class SignalHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        length = int(self.headers.get('Content-Length', 0))
        body = json.loads(self.rfile.read(length)) if length else {}
        signal = body.get('signal', '')

        if signal == 'grab_confirmed':
            print("[HOST] Sub confirmed grab")
            sub_grabbed.set()
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b'{"ok": true}')
        else:
            self.send_response(400)
            self.end_headers()
            self.wfile.write(b'{"ok": false}')

    def log_message(self, format, *args):
        pass  # suppress request logs

def start_signal_server():
    server = HTTPServer(('0.0.0.0', SIGNAL_PORT), SignalHandler)
    server.serve_forever()

def send_signal_to_sub(signal):
    """Send a coordination signal to the sub Pi."""
    url = f'http://{SUB_PI_HOST}:{SIGNAL_PORT}'
    try:
        r = requests.post(url, json={'signal': signal}, timeout=5)
        return r.status_code == 200
    except Exception as e:
        print(f"[HOST] Failed to signal sub: {e}")
        return False

# ---- Perception helpers (from ColorTracking) ----
def get_area_max_contour(contours):
    contour_area_max = 0
    area_max_contour = None
    for c in contours:
        area = math.fabs(cv2.contourArea(c))
        if area > contour_area_max:
            contour_area_max = area
            if area > 300:
                area_max_contour = c
    return area_max_contour, contour_area_max

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
    print("[HOST] Signal server listening on port", SIGNAL_PORT)

    # Init arm
    init_move()
    time.sleep(1.5)

    # Open camera
    cam = Camera.Camera()
    cam.camera_open()
    time.sleep(1)
    print("[HOST] Camera opened, detecting green block...")

    # Detection state
    size = (640, 480)
    last_x, last_y = 0, 0
    center_list = []
    count = 0
    t1 = time.time()
    start_count_t1 = True
    detected_x, detected_y = 0, 0
    rotation_angle = 0

    try:
        while True:
            frame = cam.frame
            if frame is None:
                time.sleep(0.01)
                continue

            # Show camera output on VNC
            display_img = frame.copy()
            img_h, img_w = display_img.shape[:2]
            cv2.line(display_img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
            cv2.line(display_img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
            cv2.putText(display_img, f"State: {state}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow('Host Camera', display_img)
            cv2.waitKey(1)

            if state == TransferState.DETECTING:
                img = frame.copy()
                frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
                frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
                frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

                # Detect green
                frame_mask = cv2.inRange(frame_lab,
                                         color_range[TARGET_COLOR][0],
                                         color_range[TARGET_COLOR][1])
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                contour, area = get_area_max_contour(contours)

                if area > 2500:
                    rect = cv2.minAreaRect(contour)
                    box = np.int0(cv2.boxPoints(rect))
                    roi = getROI(box)
                    img_cx, img_cy = getCenter(rect, roi, size, square_length)
                    world_x, world_y = convertCoordinate(img_cx, img_cy, size)

                    distance = math.sqrt((world_x - last_x)**2 + (world_y - last_y)**2)
                    last_x, last_y = world_x, world_y

                    if distance < 0.3:
                        center_list.extend((world_x, world_y))
                        count += 1
                        if start_count_t1:
                            start_count_t1 = False
                            t1 = time.time()
                        if time.time() - t1 > 1.5:
                            # Block is stable - record position
                            detected_x, detected_y = np.mean(
                                np.array(center_list).reshape(count, 2), axis=0)
                            rotation_angle = rect[2]
                            count = 0
                            center_list = []
                            start_count_t1 = True
                            print(f"[HOST] Green block detected at ({detected_x:.1f}, {detected_y:.1f})")
                            state = TransferState.PICKING
                    else:
                        t1 = time.time()
                        start_count_t1 = True
                        count = 0
                        center_list = []

            elif state == TransferState.PICKING:
                print("[HOST] Picking up block...")
                # Open gripper
                Board.setBusServoPulse(1, servo1 - 280, 500)
                time.sleep(0.5)

                # Set gripper rotation
                servo2_angle = getAngle(detected_x, detected_y, rotation_angle)
                Board.setBusServoPulse(2, servo2_angle, 500)
                time.sleep(0.5)

                # Move above block
                AK.setPitchRangeMoving((detected_x, detected_y - 2, 5), -90, -90, 0)
                time.sleep(1.5)

                # Lower to block
                AK.setPitchRangeMoving((detected_x, detected_y, 2), -90, -90, 0, 1000)
                time.sleep(2)

                # Close gripper
                Board.setBusServoPulse(1, servo1, 500)
                time.sleep(1)

                # Lift block
                Board.setBusServoPulse(2, 500, 500)
                AK.setPitchRangeMoving((detected_x, detected_y, 12), -90, -90, 0, 1000)
                time.sleep(1)

                print("[HOST] Block picked up, moving to handoff position...")
                state = TransferState.MOVING_TO_HANDOFF

            elif state == TransferState.MOVING_TO_HANDOFF:
                # Move to handoff position above crosshair
                # Servo 2 at 500 = default orientation (gripping sides)
                Board.setBusServoPulse(2, 500, 500)
                result = AK.setPitchRangeMoving(HANDOFF_COORD, -90, -90, 0)
                if result:
                    time.sleep(result[2] / 1000 + 0.5)
                else:
                    print("[HOST] WARNING: Handoff position unreachable, trying nearby...")
                    result = AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90)
                    if result:
                        time.sleep(result[2] / 1000 + 0.5)

                print("[HOST] At handoff position. Signaling sub...")
                state = TransferState.HOLDING

            elif state == TransferState.HOLDING:
                # Tell sub we're ready
                send_signal_to_sub('ready_to_grab')
                print("[HOST] Waiting for sub to grab...")

                # Wait for sub to confirm grab
                sub_grabbed.wait(timeout=30)
                if sub_grabbed.is_set():
                    print("[HOST] Sub has grabbed. Releasing...")
                    state = TransferState.RELEASING
                else:
                    print("[HOST] Timeout waiting for sub. Retrying signal...")
                    # Stay in HOLDING state, will re-send signal

            elif state == TransferState.RELEASING:
                # Open gripper to release block
                Board.setBusServoPulse(1, servo1 - 280, 500)
                time.sleep(1)

                # Signal sub that we've released
                send_signal_to_sub('released')
                print("[HOST] Released block. Returning to home...")
                state = TransferState.RETURNING

            elif state == TransferState.RETURNING:
                # Move arm up and away before returning home
                AK.setPitchRangeMoving((0, 10, 15), -30, -30, -90, 1000)
                time.sleep(1.5)
                init_move()
                time.sleep(1.5)
                print("[HOST] Transfer complete!")
                state = TransferState.DONE

            elif state == TransferState.DONE:
                time.sleep(1)
                break

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[HOST] Interrupted")
    finally:
        cam.camera_close()
        cv2.destroyAllWindows()
        init_move()
        time.sleep(1)

if __name__ == '__main__':
    main()
