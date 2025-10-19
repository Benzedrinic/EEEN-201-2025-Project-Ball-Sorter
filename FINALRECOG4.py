import cv2
import urllib.request
import numpy as np
import serial
import serial.tools.list_ports
import time
import math
from collections import deque

# === CAMERA & SERIAL CONFIG ===
url = 'http://192.168.76.83/capture?_cb=1759972732685'   # ESP32-CAM stream
COM_PORT = 'COM11'                                       # adjust if needed
BAUD_RATE = 115200

# === COLOR DETECTION RANGES (HSV) ===
# Red uses old calibration; yellow/blue are the same
color_ranges = {
    "yellow": ([20, 100, 100], [35, 255, 255]),
    "blue": ([95, 80, 60], [130, 255, 255]),
    "red":    ([170, 70, 50], [180, 255, 255])
}
MIN_AREA, MAX_AREA = 300, 2000
yellow_buffer = deque(maxlen=12)

# === SAFE SERIAL CONNECTION HANDLER ===
def connect_serial():
    """Attempt to open COM port safely with retries and reset buffer."""
    while True:
        try:
            ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
            time.sleep(5)  # allow Arduino reset
            ser.reset_input_buffer()
            print(f"✅ Connected to {COM_PORT}", flush=True)
            print("Python ready — waiting for Arduino to detect squash...", flush=True)
            return ser
        except serial.SerialException as e:
            print(f"⚠️ Serial connection failed: {e}")
            print("Retrying in 3 seconds...")
            time.sleep(3)

ser = connect_serial()  # initial connection

# === DOT-DETECTION FUNCTION ===
def detect_dot(mode_title="[SCANNING]"):
    """Capture a frame and return ('SB_...', frame)."""
    try:
        img_resp = urllib.request.urlopen(url, timeout=3)
        imgnp = np.array(bytearray(img_resp.read()), dtype=np.uint8)
        frame = cv2.imdecode(imgnp, -1)
        if frame is None:
            print("⚠️ Invalid frame received, skipping...", flush=True)
            return "", None
    except Exception as e:
        print("⚠️ Camera read failed:", e, flush=True)
        return "", None

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_total = np.zeros(hsv.shape[:2], dtype=np.uint8)

    # --- Red detection (no circularity filter, old calibration) ---
    lower_red = np.array(color_ranges["red"][0])
    upper_red = np.array(color_ranges["red"][1])
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    mask_total = cv2.bitwise_or(mask_total, mask_red)

    red_count = 0
    cnts, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts:
        area = cv2.contourArea(c)
        if MIN_AREA < area < MAX_AREA:
            red_count += 1
            (x, y), r = cv2.minEnclosingCircle(c)
            cv2.circle(frame, (int(x), int(y)), int(r), (0, 0, 255), 2)
            cv2.putText(frame, "red", (int(x) - 20, int(y) - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # --- Yellow and Blue detection (with circularity filter) ---
    yellow_count, blue_count = 0, 0
    for color in ["yellow", "blue"]:
        lower, upper = np.array(color_ranges[color][0]), np.array(color_ranges[color][1])
        mask = cv2.inRange(hsv, lower, upper)
        mask_total = cv2.bitwise_or(mask_total, mask)
        cnts, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            area = cv2.contourArea(c)
            if MIN_AREA < area < MAX_AREA:
                peri = cv2.arcLength(c, True)
                circ = (4 * math.pi * area) / (peri * peri + 1e-5)
                if circ > 0.6:  # fairly circular
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
                        cv2.putText(frame, color, (cx - 20, cy - 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                        if color == "yellow":
                            yellow_count += 1
                        elif color == "blue":
                            blue_count += 1

    yellow_buffer.append(yellow_count)
    stable_yellow = max(set(yellow_buffer), key=yellow_buffer.count) if yellow_buffer else 0

    # --- Determine result ---
    result = ""
    if red_count >= 1:
        result = "SB_RED"
    elif blue_count >= 1:
        result = "SB_BLUE"
    elif stable_yellow == 1:
        result = "SB_YELLOW_SINGLE"
    elif stable_yellow >= 2:
        result = "SB_YELLOW_DOUBLE"

    # --- Display frames ---
    window_title = f"Camera Feed {mode_title} → {result if result else 'Detecting...'}"
    cv2.setWindowTitle("Camera Feed", window_title)
    cv2.putText(frame, result, (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 3)
    res = cv2.bitwise_and(frame, frame, mask=mask_total)
    cv2.imshow("Camera Feed", frame)
    cv2.imshow("Mask View", mask_total)
    cv2.imshow("Detected Dots", res)

    return result, frame


# === MAIN LOOP ===
try:
    while True:
        try:
            raw = ser.readline().decode(errors="ignore").strip()
        except serial.SerialException as e:
            print(f"⚠️ Serial read error: {e}")
            ser.close()
            ser = connect_serial()
            continue

        if not raw:
            continue

        print("Arduino:", raw, flush=True)

        # === TRIGGER: Squash detected ===
        if "START_SQUASH" in raw:
            print("🔴 Squash detected! Starting dot detection...", flush=True)
            result = ""
            last_seen = ""
            frame_confirmations = 0
            start_time = time.time()
            min_runtime = 10   # seconds of visible scanning window

            print("🔍 Scanning for dots... (keep camera windows visible)", flush=True)

            while True:
                result, frame = detect_dot("[SCANNING]")

                # stability tracking
                if result != "":
                    if result == last_seen:
                        frame_confirmations += 1
                    else:
                        frame_confirmations = 1
                        last_seen = result
                else:
                    frame_confirmations = 0

                elapsed = time.time() - start_time
                # require both confirmed frames *and* min time visible
                if frame_confirmations >= 3 and elapsed >= min_runtime:
                    final_result = last_seen
                    print(f"\n🎯 Final dot color detected: {final_result}", flush=True)
                    ser.write((final_result + "\n").encode())
                    print(f"📤 Sent to Arduino: {final_result}", flush=True)

                    # === Keep camera windows open for visual confirmation ===
                    confirm_time = time.time()
                    hold_duration = 2   # seconds to inspect detected result
                    print(f"🕒 Holding camera view for {hold_duration}s to confirm...", flush=True)
                    while time.time() - confirm_time < hold_duration:
                        detect_dot(f"[RESULT: {final_result}]")  # keep refreshing window with title
                        if cv2.waitKey(100) & 0xFF == ord('q'):
                            break
                    print("✅ Returning to standby.\n", flush=True)
                    break

                if cv2.waitKey(5) & 0xFF == ord('q'):
                    raise KeyboardInterrupt

except KeyboardInterrupt:
    print("🛑 Program stopped by user.", flush=True)
finally:
    try:
        ser.close()
    except:
        pass
    cv2.destroyAllWindows()
    print("🔒 Serial closed. Windows destroyed.", flush=True)
