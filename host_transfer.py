#!/usr/bin/python3
# coding=utf8
"""
Host arm (MartyRobbins) - Dual-arm block transfer
Detects all 3 blocks (green, red, blue), picks up green,
holds at handoff position above red block at z=20,
waits for sub arm to grab, then releases.
Sends block positions (transformed to sub coords) to sub.
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
SIGNAL_PORT = 9090
TARGET_COLOR = 'green'
HANDOFF_Z = 25  # z height for handoff

# ---- Coordinate transform: host -> sub ----
# Arms are mirrored across the crosshair (y=20 in arm coords)
# sub_x = -host_x, sub_y = 40 - host_y, sub_z = host_z
def host_to_sub_coords(x, y, z):
    return (-x, 40 - y, z)

# ---- Arm setup ----
AK = ArmIK()
servo1 = 500

# ---- State machine ----
class TransferState:
    DETECTING = 0
    PICKING = 1
    MOVING_TO_HANDOFF = 2
    HOLDING = 3
    RELEASING = 4
    RETURNING = 5
    DONE = 6

state = TransferState.DETECTING
sub_grabbed = threading.Event()

# ---- Block positions (detected by camera) ----
block_positions = {}  # {'red': (x, y), 'green': (x, y), 'blue': (x, y)}

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
        pass

def start_signal_server():
    server = HTTPServer(('0.0.0.0', SIGNAL_PORT), SignalHandler)
    server.serve_forever()

def send_signal_to_sub(signal, data=None):
    """Send a coordination signal to the sub Pi."""
    url = f'http://{SUB_PI_HOST}:{SIGNAL_PORT}'
    payload = {'signal': signal}
    if data:
        payload['data'] = data
    try:
        r = requests.post(url, json=payload, timeout=5)
        return r.status_code == 200
    except Exception as e:
        print(f"[HOST] Failed to signal sub: {e}")
        return False

# ---- Perception helpers ----
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

def detect_color_block(frame_lab, color_name, size):
    """Detect a single color block, return (x, y, rect) or None."""
    frame_mask = cv2.inRange(frame_lab, color_range[color_name][0], color_range[color_name][1])
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
        return world_x, world_y, rect, box
    return None

# ---- Init ----
def init_move():
    Board.setBusServoPulse(1, servo1 - 50, 300)
    Board.setBusServoPulse(2, 500, 500)
    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

# ---- Main logic ----
def main():
    global state, block_positions

    sig_thread = threading.Thread(target=start_signal_server, daemon=True)
    sig_thread.start()
    print("[HOST] Signal server listening on port", SIGNAL_PORT)

    init_move()
    time.sleep(1.5)

    cam = Camera.Camera()
    cam.camera_open()
    time.sleep(1)
    print("[HOST] Camera opened, detecting blocks...")

    size = (640, 480)
    range_rgb = {'red': (0, 0, 255), 'green': (0, 255, 0), 'blue': (255, 0, 0)}

    # Green block tracking state
    last_x, last_y = 0, 0
    center_list = []
    count = 0
    t1 = time.time()
    start_count_t1 = True
    detected_x, detected_y = 0, 0
    rotation_angle = 0

    # Handoff coordinate (will be set to above red block)
    handoff_coord = None

    try:
        while True:
            frame = cam.frame
            if frame is None:
                time.sleep(0.01)
                continue

            img = frame.copy()
            frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
            frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
            frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

            # Show camera output on VNC
            display_img = frame.copy()
            img_h, img_w = display_img.shape[:2]
            cv2.line(display_img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
            cv2.line(display_img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

            state_names = {0: "DETECTING", 1: "PICKING", 2: "MOVE_HANDOFF",
                           3: "HOLDING", 4: "RELEASING", 5: "RETURNING", 6: "DONE"}
            cv2.putText(display_img, f"State: {state_names.get(state, state)}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Always detect all blocks for display
            for color_name in ['red', 'green', 'blue']:
                result = detect_color_block(frame_lab, color_name, size)
                if result:
                    wx, wy, rect, box = result
                    block_positions[color_name] = (wx, wy)
                    cv2.drawContours(display_img, [box], -1, range_rgb[color_name], 2)
                    cv2.putText(display_img, f"{color_name}({wx:.1f},{wy:.1f})",
                                (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, range_rgb[color_name], 1)

            cv2.imshow('Host Camera', display_img)
            cv2.waitKey(1)

            if state == TransferState.DETECTING:
                # Focus on green block stability
                green = detect_color_block(frame_lab, 'green', size)
                if green:
                    world_x, world_y, rect, box = green
                    distance = math.sqrt((world_x - last_x)**2 + (world_y - last_y)**2)
                    last_x, last_y = world_x, world_y

                    if distance < 0.3:
                        center_list.extend((world_x, world_y))
                        count += 1
                        if start_count_t1:
                            start_count_t1 = False
                            t1 = time.time()
                        if time.time() - t1 > 1.5:
                            detected_x, detected_y = np.mean(
                                np.array(center_list).reshape(count, 2), axis=0)
                            rotation_angle = rect[2]
                            count = 0
                            center_list = []
                            start_count_t1 = True

                            # Check we have red and blue positions too
                            if 'red' in block_positions and 'blue' in block_positions:
                                red_x, red_y = block_positions['red']
                                handoff_coord = (red_x, red_y, HANDOFF_Z)
                                print(f"[HOST] Green at ({detected_x:.1f}, {detected_y:.1f})")
                                print(f"[HOST] Red at ({red_x:.1f}, {red_y:.1f})")
                                print(f"[HOST] Blue at ({block_positions['blue'][0]:.1f}, {block_positions['blue'][1]:.1f})")
                                print(f"[HOST] Handoff will be above red at {handoff_coord}")
                                state = TransferState.PICKING
                            else:
                                print("[HOST] Green found but waiting for red/blue detection...")
                                count = 0
                                center_list = []
                                start_count_t1 = True
                    else:
                        t1 = time.time()
                        start_count_t1 = True
                        count = 0
                        center_list = []

            elif state == TransferState.PICKING:
                print("[HOST] Picking up green block...")
                Board.setBusServoPulse(1, servo1 - 280, 500)
                time.sleep(0.5)

                servo2_angle = getAngle(detected_x, detected_y, rotation_angle)
                Board.setBusServoPulse(2, servo2_angle, 500)
                time.sleep(0.5)

                AK.setPitchRangeMoving((detected_x, detected_y - 2, 5), -90, -90, 0)
                time.sleep(1.5)

                AK.setPitchRangeMoving((detected_x, detected_y, 1), -90, -90, 0, 1000)
                time.sleep(2)

                Board.setBusServoPulse(1, servo1, 500)
                time.sleep(1)

                Board.setBusServoPulse(2, 500, 500)
                AK.setPitchRangeMoving((detected_x, detected_y, 12), -90, -90, 0, 1000)
                time.sleep(1)

                print("[HOST] Block picked up, moving to handoff above red...")
                state = TransferState.MOVING_TO_HANDOFF

            elif state == TransferState.MOVING_TO_HANDOFF:
                Board.setBusServoPulse(2, 500, 500)
                result = AK.setPitchRangeMoving(handoff_coord, -90, -90, 0)
                if result:
                    time.sleep(result[2] / 1000 + 0.5)
                else:
                    print("[HOST] WARNING: Handoff position unreachable!")
                    # Try with wider pitch range
                    result = AK.setPitchRangeMoving(handoff_coord, -30, -30, -90)
                    if result:
                        time.sleep(result[2] / 1000 + 0.5)

                print("[HOST] At handoff position. Signaling sub...")
                state = TransferState.HOLDING

            elif state == TransferState.HOLDING:
                # Send block positions transformed to sub coordinates
                sub_handoff = host_to_sub_coords(*handoff_coord)
                sub_blue = host_to_sub_coords(block_positions['blue'][0],
                                               block_positions['blue'][1], 0)
                data = {
                    'handoff': list(sub_handoff),
                    'blue': list(sub_blue),
                }
                send_signal_to_sub('ready_to_grab', data)
                print(f"[HOST] Sent to sub - handoff: {sub_handoff}, blue: {sub_blue}")
                print("[HOST] Waiting for sub to grab...")

                sub_grabbed.wait(timeout=30)
                if sub_grabbed.is_set():
                    print("[HOST] Sub has grabbed. Releasing...")
                    state = TransferState.RELEASING
                else:
                    print("[HOST] Timeout waiting for sub. Retrying signal...")

            elif state == TransferState.RELEASING:
                Board.setBusServoPulse(1, servo1 - 280, 500)
                time.sleep(1)

                send_signal_to_sub('released')
                print("[HOST] Released block. Returning to home...")
                state = TransferState.RETURNING

            elif state == TransferState.RETURNING:
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
