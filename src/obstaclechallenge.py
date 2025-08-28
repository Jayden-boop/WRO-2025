import cv2
import numpy as np
import time
import ros_robot_controller_sdk as rrc
import math
import json
import os
from picamera2 import Picamera2
import RPi.GPIO as GPIO

# Imports for LD19 LIDAR parsing
import serial
import struct

board = rrc.Board()

MID_SERVO = 63
MAX_TURN_DEGREE = 33
DC_SPEED = 1375
PILLAR_SIZE = 400
PD = 0.004
PG = 0.004
GYROPG = 1
ROI_LEFT = [0, 210, 100, 470]
ROI_RIGHT = [540, 210, 640, 470]
ROI_LINE = [240, 370, 400, 400]
ROI_MIDDLE = [0, 200, 640, 400]

ROI_FRONT = [280, 240, 360, 280]


LOWER_BLACK_THRESHOLD = np.array([0, 0, 0])
UPPER_BLACK_THRESHOLD = np.array([180, 255, 90])

LOWER_RED_THRESHOLD1 = np.array([0, 154, 50])
UPPER_RED_THRESHOLD1 = np.array([6, 255, 255])
LOWER_RED_THRESHOLD2 = np.array([175, 180, 50])
UPPER_RED_THRESHOLD2 = np.array([180, 255, 255])

LOWER_GREEN_THRESHOLD = np.array([65, 90, 35])
UPPER_GREEN_THRESHOLD = np.array([85, 255, 185])

LOWER_ORANGE_THRESHOLD1 = np.array([180, 80, 120])
UPPER_ORANGE_THRESHOLD1 = np.array([180, 255, 255])
LOWER_ORANGE_THRESHOLD2 = np.array([5, 156, 120])
UPPER_ORANGE_THRESHOLD2 = np.array([20, 255, 255])

LOWER_BLUE_THRESHOLD = np.array([105, 45, 70])
UPPER_BLUE_THRESHOLD = np.array([130, 255, 200])


WIDTH = 640
HEIGHT = 480
POINTS = [(115, 200), (525, 200), (640, 370), (0, 370)]

MAX_TURNS = 4
ACTIONS_TO_STRAIGHT = 120
WALL_THRESHOLD = 200
NO_WALL_THRESHOLD = 200
TURN_ITER_LIMIT = 30
LINE_THRESHOLD = 100
SHOW_CONTOURS = True
action_counter = 0
turn_counter = 0
turn_length_counter = 0

close_to_wall = False
right_reading = True
right_reading_2 = True
parking_left = False
startFromParkingLot = True
track_dir = None
done_turning = False
turn_dir = None
last_difference = 0
current_difference = 0
prev_pillar_error = 0
seen_line = False
line_end = False
servo_angle = 0
exit_parking_lot = False
exit_parking_lot_right = False
exit_parking_lot_left = False
exit_parking_lot_red = False
exit_parking_lot_green = False
avoid_phase = False
im = None
image = None
turn_to_parking_section = False
red_target = 120
green_target = 520
gyro = False
at_parking_lot = False
clear = False
stopMove = False
running = True
forward = True
parking_right = False
passed_wall = False

# Add these with your other global variables

previous_angle = None
current_scan_points = []

rotation_count = 0
shared_folder = "/home/pi/docker/tmp"  # Raspberry Pi path
filepath = os.path.join(shared_folder, "imu_data.json")

last_modified = 0


picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 30
picam2.configure("preview")
picam2.start()

fps_counter = 0
fps_start_time = time.time()
fps = 0

# LD19 LIDAR Constants
PORT = "/dev/ttyAMA0"
BAUD = 230400
PACKET_HEADER = 0x54
PACKET_LEN = 47

# Initialize serial for LD19
ser = serial.Serial(PORT, BAUD, timeout=0.01)  # Reduced timeout for non-blocking read
buffer = bytearray()


def find_packet_start(buffer):
    """Find the start index of a valid packet in buffer."""
    for i in range(len(buffer) - 1):
        if buffer[i] == 0x54 and (buffer[i + 1] & 0xFF) == 0x2C:
            return i
    return -1


def parse_packet(packet):
    if len(packet) != PACKET_LEN:
        return None
    speed = struct.unpack_from("<H", packet, 2)[0] / 64.0  # RPM
    start_angle = struct.unpack_from("<H", packet, 4)[0] / 100.0
    measurements = []
    for i in range(12):
        offset = 6 + i * 3
        dist = struct.unpack_from("<H", packet, offset)[0]
        confidence = packet[offset + 2]
        measurements.append((dist, confidence))
    end_angle = struct.unpack_from("<H", packet, 42)[0] / 100.0
    timestamp = struct.unpack_from("<H", packet, 44)[0]
    crc = packet[46]
    return {
        "speed": speed,
        "start_angle": start_angle,
        "end_angle": end_angle,
        "timestamp": timestamp,
        "crc": crc,
        "points": measurements,
    }


def fit_line_least_squares(points):
    if len(points) < 2:
        return None, None
    x_coords = [p[0] for p in points]
    y_coords = [p[1] for p in points]
    n = len(points)
    sum_y, sum_x = sum(y_coords), sum(x_coords)
    sum_xy = sum(x * y for x, y in points)
    sum_yy = sum(y * y for y in y_coords)
    denominator = n * sum_yy - sum_y * sum_y
    if abs(denominator) < 1e-10:
        return None, None
    slope = (n * sum_xy - sum_x * sum_y) / denominator
    intercept = (sum_x - slope * sum_y) / n
    return slope, intercept


def calculate_wall_slope_and_error(wall_points):
    if len(wall_points) < 8:
        return None, None, "Insufficient points"
    slope, intercept = fit_line_least_squares(wall_points)
    if slope is None:
        return None, None, "Could not fit line"
    angle_from_vertical = math.degrees(math.atan(slope))
    return slope, angle_from_vertical, "OK"


def interpolate_angles(start, end, count):
    """Return a list of interpolated angles from start to end over count points."""
    angle_range = (end - start + 360) % 360
    step = angle_range / (count - 1)
    return [(start + i * step) % 360 for i in range(count)]


def pwm(degree):
    return round(degree * 11.1 + 500)


def drawROI(ROI):
    image = cv2.line(im, (ROI[0], ROI[1]), (ROI[2], ROI[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI[0], ROI[1]), (ROI[0], ROI[3]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI[2], ROI[3]), (ROI[2], ROI[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI[2], ROI[3]), (ROI[0], ROI[3]), (0, 255, 255), 4)


def stop():

    board.pwm_servo_set_position(0.04, [[2, 1500]])
    board.pwm_servo_set_position(0.04, [[1, pwm(MID_SERVO)]])
    time.sleep(0.1)
    board.pwm_servo_set_position(0.04, [[2, 1500]])

    cv2.destroyAllWindows()


def findMaxContour(contours):
    max_area = 0
    for i in range(len(contours)):
        area = cv2.contourArea(contours[i])
        max_area = max(area, max_area)
    return max_area


def detect_angle_reset(current_angle, previous_angle):
    if previous_angle is None:
        return False
    return previous_angle > 300 and current_angle < 60


def filter_wall_points(points, x_min=20, x_max=100, y_min=-25, y_max=0):
    wall_points = []
    for x, y in points:
        if x_min <= x <= x_max and y_min <= y <= y_max:
            wall_points.append((x, y))
    return wall_points


def imuData():
    global last_modified
    try:
        # Check if file exists and was updated
        if os.path.exists(filepath):
            current_modified = os.path.getmtime(filepath)

            if current_modified > last_modified:
                with open(filepath, "r") as f:
                    imu_data = json.load(f)

                # Process your data
                rpy_deg = imu_data["rpy_degrees"]
                angle = rpy_deg["yaw"]
                if angle < 0:
                    angle += 360

                last_modified = current_modified
                return angle

        else:
            return None

    except FileNotFoundError:
        return None
    except json.JSONDecodeError:
        return None
    except Exception as e:
        return None


board.pwm_servo_set_position(0.04, [[1, pwm(MID_SERVO)]])
board.pwm_servo_set_position(0.04, [[2, 1500]])
time.sleep(0.5)
starting_angle = None
while starting_angle == None:
    starting_angle = imuData()

print(starting_angle)
if not startFromParkingLot:
    board.pwm_servo_set_position(0.04, [[2, DC_SPEED]])
a = False
while True:

    if (
        exit_parking_lot_green
        or exit_parking_lot_red
        or exit_parking_lot_left
        or exit_parking_lot_right
    ):
        exit_parking_lot = True
    else:
        exit_parking_lot = False

    im = picam2.capture_array()
    input = np.float32(POINTS)
    output = np.float32(
        [(0, 0), (WIDTH - 1, 0), (WIDTH - 1, HEIGHT - 1), (0, HEIGHT - 1)]
    )

    img_hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

    img_thresh_black = cv2.inRange(
        img_hsv, LOWER_BLACK_THRESHOLD, UPPER_BLACK_THRESHOLD
    )

    img_thresh_green = cv2.inRange(
        img_hsv, LOWER_GREEN_THRESHOLD, UPPER_GREEN_THRESHOLD
    )
    img_thresh_red = cv2.bitwise_or(
        cv2.inRange(img_hsv, LOWER_RED_THRESHOLD1, UPPER_RED_THRESHOLD1),
        cv2.inRange(img_hsv, LOWER_RED_THRESHOLD2, UPPER_RED_THRESHOLD2),
    )
    img_thresh_blue = cv2.inRange(img_hsv, LOWER_BLUE_THRESHOLD, UPPER_BLUE_THRESHOLD)
    img_thresh_orange = cv2.bitwise_or(
        cv2.inRange(img_hsv, LOWER_ORANGE_THRESHOLD1, UPPER_ORANGE_THRESHOLD1),
        cv2.inRange(img_hsv, LOWER_ORANGE_THRESHOLD2, UPPER_ORANGE_THRESHOLD2),
    )

    left_contours_bot, hierarchy = cv2.findContours(
        img_thresh_black[ROI_LEFT[1] : ROI_LEFT[3], ROI_LEFT[0] : ROI_LEFT[2]],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )
    right_contours_bot, hierarchy = cv2.findContours(
        img_thresh_black[ROI_RIGHT[1] : ROI_RIGHT[3], ROI_RIGHT[0] : ROI_RIGHT[2]],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )

    contours_red, _ = cv2.findContours(
        img_thresh_red[ROI_MIDDLE[1] : ROI_MIDDLE[3], ROI_MIDDLE[0] : ROI_MIDDLE[2]],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )
    contours_green, _ = cv2.findContours(
        img_thresh_green[ROI_MIDDLE[1] : ROI_MIDDLE[3], ROI_MIDDLE[0] : ROI_MIDDLE[2]],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )

    contours_blue, _ = cv2.findContours(
        img_thresh_blue[ROI_LINE[1] : ROI_LINE[3], ROI_LINE[0] : ROI_LINE[2]],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    contours_orange, _ = cv2.findContours(
        img_thresh_orange[ROI_LINE[1] : ROI_LINE[3], ROI_LINE[0] : ROI_LINE[2]],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    left_contours_black, hierarchy = cv2.findContours(
        img_thresh_black[ROI_LEFT[1] : ROI_LEFT[3], ROI_LEFT[0] : ROI_LEFT[2]],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )
    right_contours_black, hierarchy = cv2.findContours(
        img_thresh_black[ROI_RIGHT[1] : ROI_RIGHT[3], ROI_RIGHT[0] : ROI_RIGHT[2]],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )

    front_contours_black, hierarchy = cv2.findContours(
        img_thresh_black[ROI_FRONT[1] : ROI_FRONT[3], ROI_FRONT[0] : ROI_FRONT[2]],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )
    contours, hierarchy = cv2.findContours(
        img_thresh_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
    )

    left_area = 0
    right_area = 0
    green_pillar_area = 0
    red_pillar_area = 0
    blue_line_area = 0
    orange_line_area = 0

    contours_coloured = [
        (contours_red, (0, 0, 255), 1, ROI_MIDDLE),
        (contours_green, (0, 255, 0), 1, ROI_MIDDLE),
        (contours, (0, 255, 255), 1, (0, 0)),
        (contours_orange, (52, 140, 235), 0, ROI_LINE),
        (contours_blue, (235, 67, 52), 0, ROI_LINE),
    ]

    left_area_black = findMaxContour(left_contours_black)
    right_area_black = findMaxContour(right_contours_black)
    front_area_black = findMaxContour(front_contours_black)
    left_area = left_area_black
    right_area = right_area_black

    for i in range(len(contours_orange)):
        cnt = contours_orange[i]
        orange_line_area = max(cv2.contourArea(cnt), orange_line_area)
        cnt[:, :, 0] += ROI_LINE[0]
        cnt[:, :, 1] += ROI_LINE[1]
        if SHOW_CONTOURS:

            cv2.drawContours(im, contours_orange, i, (255, 255, 0), 1)

    for i in range(len(contours_blue)):
        cnt = contours_blue[i]
        blue_line_area = max(cv2.contourArea(cnt), blue_line_area)
        cnt[:, :, 0] += ROI_LINE[0]
        cnt[:, :, 1] += ROI_LINE[1]
        if SHOW_CONTOURS:

            cv2.drawContours(im, contours_blue, i, (255, 255, 0), 1)

    for c in contours_coloured:
        cont = c[0]
        colour = c[1]
        ROIROI = c[3]
        for i, cnt in enumerate(cont):
            area = cv2.contourArea(cnt)
            if area > 100:
                cnt[:, :, 0] += ROIROI[0]
                cnt[:, :, 1] += ROIROI[1]
                if SHOW_CONTOURS:

                    cv2.drawContours(im, [cnt], -1, colour, 2)
    if startFromParkingLot:
        print("fajf " + str(right_area) + " " + str(left_area))

        if left_area > right_area:
            track_dir = "right"

            exit_parking_lot_right = True
            startFromParkingLot = False
        else:
            track_dir = "left"
            exit_parking_lot_left = True
            startFromParkingLot = False

    if exit_parking_lot_left:
        pw = pwm(0)

        board.pwm_servo_set_position(0.04, [[1, pw]])
        time.sleep(0.4)
        board.pwm_servo_set_position(0.04, [[2, DC_SPEED + 5]])

        time.sleep(0.7)
        """
        pw = pwm(MID_SERVO)

        board.pwm_servo_set_position(0.04, [[1, pw]])

        board.pwm_servo_set_position(0.04, [[2, DC_SPEED + 5]])

        time.sleep(1)

        pw = pwm(0)

        board.pwm_servo_set_position(0.04, [[1, pw]])

        board.pwm_servo_set_position(0.04, [[2, 1620]])

        time.sleep(2.5)

        pw = pwm(MID_SERVO)
        board.pwm_servo_set_position(0.04, [[1, pw]])
        time.sleep(0.5)
        
        
        """

        exit_parking_lot_left = False

        board.pwm_servo_set_position(0.04, [[2, DC_SPEED]])

        turn_dir = None
        closest_pillar_distance = 10000
        closest_pillar_colour = None
        closest_pillar_area = -1

        print("aaaa" + str(closest_pillar_area) + " " + str(closest_pillar_distance))

    if exit_parking_lot_right:
        pw = pwm(MID_SERVO + MAX_TURN_DEGREE)
        board.pwm_servo_set_position(0.04, [[1, pw]])

        board.pwm_servo_set_position(0.04, [[2, DC_SPEED + 7]])
        time.sleep(1.1)

        pw = pwm(MID_SERVO)
        board.pwm_servo_set_position(0.04, [[1, pw]])
        board.pwm_servo_set_position(0.04, [[2, DC_SPEED]])
        time.sleep(0.7)

        exit_parking_lot_right = False
        turn_dir = None

        closest_pillar_distance = 10000
        closest_pillar_colour = None
        closest_pillar_area = -1
        if not a:
            a = True

            continue

    num_pillars_r = 0
    num_pillars_g = 0

    OBSTACLEPG = 0.0013
    OBSTACLEPD = 0.0013
    YAXISPG = 0
    x, y, w, h = 0, 0, 0, 0

    closest_pillar_distance = 100000
    closest_pillar_x = None
    closest_pillar_y = None
    closest_pillar_area = None
    closest_pillar_colour = None

    for i in contours_red:
        area = cv2.contourArea(i)

        if area > PILLAR_SIZE:
            approx = cv2.approxPolyDP(i, 0.01 * cv2.arcLength(i, True), True)
            x, y, w, h = cv2.boundingRect(approx)

            pillar_distance = math.dist([x + w // 2, y], [320, 480])

            if pillar_distance < 500:
                if SHOW_CONTOURS:

                    image = cv2.line(im, (x, y), (x + w, y), (0, 255, 255), 1)
                    image = cv2.line(im, (x, y), ((x, h + y)), (0, 255, 255), 1)
                    image = cv2.line(im, (x + w, y), (x + w, y + h), (0, 255, 255), 1)
                    image = cv2.line(im, (x, y + h), (x + w, y + h), (0, 255, 255), 1)
                    cv2.circle(im, (int(x + (w / 2)), y), 5, (255, 255, 0), 1, -1)

                num_pillars_r += 1

                if pillar_distance < closest_pillar_distance:
                    if y + h > 450 and x + (w // 2) < red_target:
                        prevPillar = "red"
                        pass
                    elif y + h < 125:
                        print(y + h)
                        pass
                    else:
                        closest_pillar_distance = pillar_distance
                        closest_pillar_colour = "red"
                        closest_pillar_x = x + w // 2
                        closest_pillar_y = y
                        closest_pillar_area = h * w

    for i in contours_green:
        area = cv2.contourArea(i)

        if area > PILLAR_SIZE:
            approx = cv2.approxPolyDP(i, 0.01 * cv2.arcLength(i, True), True)
            x, y, w, h = cv2.boundingRect(approx)

            pillar_distance = math.dist([x + w // 2, y], [320, 480])

            if pillar_distance < 500:
                if SHOW_CONTOURS:
                    image = cv2.line(im, (x, y), (x + w, y), (0, 255, 255), 1)
                    image = cv2.line(im, (x, y), ((x, h + y)), (0, 255, 255), 1)
                    image = cv2.line(im, (x + w, y), (x + w, y + h), (0, 255, 255), 1)
                    image = cv2.line(im, (x, y + h), (x + w, y + h), (0, 255, 255), 1)
                    cv2.circle(im, (int(x + (w / 2)), y), 5, (255, 255, 0), 1, -1)

                num_pillars_g += 1

                if pillar_distance < closest_pillar_distance:
                    if y + h > 450 and (x + (w // 2) > green_target):
                        print("hello")
                        prevPillar = "green"
                        pass
                    elif y + h < 125:
                        print("hi")

                        pass
                    else:
                        closest_pillar_distance = pillar_distance
                        closest_pillar_colour = "green"
                        closest_pillar_x = x + w // 2
                        closest_pillar_y = y
                        closest_pillar_area = h * w

    if turn_counter >= MAX_TURNS:
        if track_dir == "left":
            parking_left = True
        if track_dir == "right":
            parking_right = True
    if blue_line_area >= LINE_THRESHOLD and track_dir != "right" and seen_line == False:
        track_dir = "left"
        turn_dir = "left"
        seen_line = True
        turn_counter += 1
    if (
        orange_line_area >= LINE_THRESHOLD
        and track_dir != "left"
        and seen_line == False
    ):
        track_dir = "right"
        turn_dir = "right"
        seen_line = True
        turn_counter += 1

    if turn_dir == "left":
        if orange_line_area >= LINE_THRESHOLD:
            line_end = True

    if turn_dir == "right":
        if blue_line_area >= LINE_THRESHOLD:
            line_end = True

    if closest_pillar_colour == None:
        target = None
    elif closest_pillar_colour == "red":
        target = red_target

    elif closest_pillar_colour == "green":
        target = green_target

    if target != None:
        error = target - closest_pillar_x
        servo_angle = (
            MID_SERVO
            - ((((error) * MAX_TURN_DEGREE) * OBSTACLEPG))
            - (error - prev_pillar_error) * OBSTACLEPD
        )
        prev_pillar_error = error
        if closest_pillar_colour == "green":
            servo_angle -= closest_pillar_y * YAXISPG

            if (
                # if the pillar is too close to the robot and not on the right side, reverse the robot
                closest_pillar_area > 7000
                and (closest_pillar_x) < 390
                and closest_pillar_distance < 300
                and not exit_parking_lot
                and not (parking_right or parking_right)
            ):
                servo_angle = MID_SERVO  # set the servo to straight
                pw = pwm(servo_angle)

                board.pwm_servo_set_position(0.04, [[2, 1500]])
                board.pwm_servo_set_position(0.04, [[1, pw]])

                board.pwm_servo_set_position(0.04, [[2, 1620]])
                time.sleep(1)
                board.pwm_servo_set_position(0.04, [[2, DC_SPEED]])

        if closest_pillar_colour == "red":
            servo_angle += closest_pillar_y * YAXISPG

            if (
                # if the pillar is too close to the robot and not on the right side, reverse the robot
                closest_pillar_area > 7000
                and (closest_pillar_x) > 250
                and closest_pillar_distance < 300
                and not exit_parking_lot
                and not (parking_right or parking_right)
            ):
                servo_angle = MID_SERVO  # set the servo to straight
                pw = pwm(servo_angle)

                board.pwm_servo_set_position(0.04, [[2, 1500]])
                board.pwm_servo_set_position(0.04, [[1, pw]])

                board.pwm_servo_set_position(0.04, [[2, 1620]])
                time.sleep(1)
                board.pwm_servo_set_position(0.04, [[2, DC_SPEED]])

    else:
        if (
            blue_line_area >= LINE_THRESHOLD
            and track_dir != "right"
            and seen_line == False
        ):
            track_dir = "left"

            turn_dir = "left"
        if (
            orange_line_area >= LINE_THRESHOLD
            and track_dir != "left"
            and seen_line == False
        ):
            track_dir = "right"

            turn_dir = "right"

        if turn_dir == "right":
            servo_angle = MID_SERVO + (MAX_TURN_DEGREE * 0.2)
        elif turn_dir == "left":
            servo_angle = MID_SERVO - (MAX_TURN_DEGREE * 0.3)
        elif turn_dir == None:
            current_difference = left_area - right_area

            servo_angle = MID_SERVO + (
                current_difference * PG + (current_difference - last_difference) * PD
            )

    if turn_dir == "left" and not blue_line_area >= LINE_THRESHOLD:

        if left_area > WALL_THRESHOLD and line_end:
            turn_dir = None
            line_end = False
            seen_line = False

    elif turn_dir == "right" and not orange_line_area >= LINE_THRESHOLD:

        if right_area > WALL_THRESHOLD and line_end:
            turn_dir = None
            line_end = False

            seen_line = False
    last_difference = current_difference
    """
    if left_area > 14000:
        servo_angle = MID_SERVO + MAX_TURN_DEGREE
    if right_area > 14000:
        servo_angle = MID_SERVO - MAX_TURN_DEGREE

    
    
    """

    if (
        track_dir == "left"
        and left_area == 0
        and right_area > WALL_THRESHOLD
        and target == None
    ):
        servo_angle = MID_SERVO - MAX_TURN_DEGREE

    if (
        track_dir == "right"
        and right_area == 0
        and left_area > WALL_THRESHOLD
        and target == None
    ):
        servo_angle = MID_SERVO + MAX_TURN_DEGREE

    if (
        track_dir == "left"
        and front_area_black > 200
        and target == None
        and left_area > 0
    ):
        if turn_counter == 0:
            servo_angle = MID_SERVO + MAX_TURN_DEGREE + 10
        elif right_area > 1500:
            servo_angle = MID_SERVO - MAX_TURN_DEGREE

    if (
        track_dir == "right"
        and right_area > 1500
        and front_area_black > 200
        and target == None
    ):
        if turn_counter == 0:
            servo_angle = MID_SERVO - MAX_TURN_DEGREE
        elif left_area > 1500:
            servo_angle = MID_SERVO + MAX_TURN_DEGREE
    if (
        track_dir == "left"
        and target == green_target
        and left_area > 6500
        and closest_pillar_distance > 300
    ):
        servo_angle = MID_SERVO + MAX_TURN_DEGREE

    if (
        track_dir == "right"
        and target == red_target
        and right_area > 6500
        and closest_pillar_distance > 300
    ):
        servo_angle = MID_SERVO - MAX_TURN_DEGREE

    if parking_left:
        print("parking")
        if clear == False:
            servo_angle = MID_SERVO + (MAX_TURN_DEGREE)
            board.pwm_servo_set_position(0.04, [[2, DC_SPEED]])
            board.pwm_servo_set_position(0.04, [[1, pwm(servo_angle)]])
            time.sleep(0.3)
            servo_angle = MID_SERVO
            board.pwm_servo_set_position(0.04, [[2, DC_SPEED]])
            board.pwm_servo_set_position(0.04, [[1, pwm(servo_angle)]])
            time.sleep(1)

            clear = True
            board.pwm_servo_set_position(0.04, [[2, DC_SPEED]])

        current_scan_points = []
        previous_angle = None
        rotation_count = 0

        ser.reset_input_buffer()
        buffer = b""
        board.pwm_servo_set_position(0.04, [[2, DC_SPEED + 25]])

        while not close_to_wall:

            try:
                bytes_to_read = ser.in_waiting
                if bytes_to_read > 0:
                    buffer += ser.read(bytes_to_read)

                while len(buffer) >= PACKET_LEN:
                    idx = find_packet_start(buffer)
                    if idx == -1:
                        buffer = buffer[-(PACKET_LEN - 1) :]
                        break
                    if len(buffer) - idx < PACKET_LEN:
                        buffer = buffer[idx:]
                        break

                    packet = buffer[idx : idx + PACKET_LEN]
                    parsed = parse_packet(packet)

                    if parsed:
                        angles = interpolate_angles(
                            parsed["start_angle"], parsed["end_angle"], 12
                        )
                        for (dist, conf), angle in zip(parsed["points"], angles):
                            if detect_angle_reset(angle, previous_angle):
                                if len(current_scan_points) > 50:
                                    rotation_count += 1

                                    wall_points = filter_wall_points(
                                        current_scan_points,
                                        x_min=-4,
                                        x_max=4,
                                        y_min=-100,
                                        y_max=-5,
                                    )

                                    if len(wall_points) >= 8:
                                        slope, angle_error, status = (
                                            calculate_wall_slope_and_error(wall_points)
                                        )
                                        if not slope == None:
                                            angle_error = math.degrees(
                                                math.atan(1 / slope)
                                            )
                                        if slope is not None:
                                            # This is the main output for the test
                                            print(
                                                f"Rotation #{rotation_count}: Angle Error = {angle_error:.2f}°"
                                            )
                                            print(slope)
                                            servo_angle = MID_SERVO + (
                                                angle_error * GYROPG
                                            )

                                        else:
                                            print(
                                                f"Rotation #{rotation_count}: Could not calculate angle."
                                            )
                                    else:
                                        print(
                                            f"Rotation #{rotation_count}: Insufficient wall points."
                                        )

                                current_scan_points = []

                            if 0 < dist < 1000:
                                angle_rad = math.radians(angle)
                                x = dist * math.cos(angle_rad) / 10.0
                                y = dist * math.sin(angle_rad) / 10.0

                                if y > -21 and y < -19 and x > -3 and x < 3:
                                    close_to_wall = True

                                current_scan_points.append((x, y))

                            previous_angle = angle

                    buffer = buffer[idx + PACKET_LEN :]
                if (servo_angle) > MID_SERVO + MAX_TURN_DEGREE:
                    servo_angle = MID_SERVO + MAX_TURN_DEGREE
                elif (servo_angle) < MID_SERVO - MAX_TURN_DEGREE:
                    servo_angle = MID_SERVO - MAX_TURN_DEGREE
                time.sleep(0.02)
                board.pwm_servo_set_position(0.02, [[1, pwm(servo_angle)]])
                time.sleep(0.02)

            except Exception as e:
                print(f"An error occurred: {e}")

            except KeyboardInterrupt:
                print("\nProgram ended.")
                board.pwm_servo_set_position(0.04, [[2, 1500]])
                board.pwm_servo_set_position(0.04, [[1, pwm(MID_SERVO)]])
                ser.close()
                break

        if not turn_to_parking_section:
            print("HELLOOOOOOOO??????")
            servo_angle = MID_SERVO + MAX_TURN_DEGREE
            board.pwm_servo_set_position(0.04, [[2, 1500]])

            board.pwm_servo_set_position(0.04, [[1, pwm(servo_angle)]])
            time.sleep(0.5)
            servo_angle = MID_SERVO + MAX_TURN_DEGREE
            board.pwm_servo_set_position(0.04, [[2, 1620]])

            board.pwm_servo_set_position(0.04, [[1, pwm(servo_angle)]])
            time.sleep(1.8)
            board.pwm_servo_set_position(0.04, [[2, 1500]])

            turn_to_parking_section = True

        else:

            print("AINFSAF")

            current_scan_points = []
            previous_angle = None
            rotation_count = 0

            ser.reset_input_buffer()
            buffer = b""
            board.pwm_servo_set_position(0.04, [[1, pwm(MID_SERVO)]])

            board.pwm_servo_set_position(0.04, [[2, 1500]])

            time.sleep(0.5)
            board.pwm_servo_set_position(0.04, [[2, DC_SPEED + 25]])

            while right_reading:

                try:
                    bytes_to_read = ser.in_waiting
                    if bytes_to_read > 0:
                        buffer += ser.read(bytes_to_read)

                    while len(buffer) >= PACKET_LEN:
                        idx = find_packet_start(buffer)
                        if idx == -1:
                            buffer = buffer[-(PACKET_LEN - 1) :]
                            break
                        if len(buffer) - idx < PACKET_LEN:
                            buffer = buffer[idx:]
                            break

                        packet = buffer[idx : idx + PACKET_LEN]
                        parsed = parse_packet(packet)

                        if parsed:
                            angles = interpolate_angles(
                                parsed["start_angle"], parsed["end_angle"], 12
                            )
                            for (dist, conf), angle in zip(parsed["points"], angles):
                                if detect_angle_reset(angle, previous_angle):
                                    if len(current_scan_points) > 50:
                                        rotation_count += 1
                                        wall_points = filter_wall_points(
                                            current_scan_points,
                                            x_min=5,
                                            x_max=100,
                                            y_min=-14,
                                            y_max=-2,
                                        )

                                        if len(wall_points) >= 4:
                                            slope, angle_error, status = (
                                                calculate_wall_slope_and_error(
                                                    wall_points
                                                )
                                            )
                                            if slope is not None:
                                                # This is the main output for the test
                                                print(
                                                    f"Rotation #{rotation_count}: Angle Error = {angle_error:.2f}°"
                                                )
                                                print(servo_angle)
                                                servo_angle = MID_SERVO - (
                                                    angle_error * GYROPG
                                                )

                                            else:
                                                print(
                                                    f"Rotation #{rotation_count}: Could not calculate angle."
                                                )
                                        else:
                                            servo_angle = MID_SERVO + 20
                                            print(
                                                f"Rotation #{rotation_count}: Insufficient wall points.aaaaa"
                                            )

                                            wall_points = filter_wall_points(
                                                current_scan_points,
                                                x_min=5,
                                                x_max=100,
                                                y_min=-8,
                                                y_max=-1,
                                            )

                                            if len(wall_points) >= 4:
                                                slope, angle_error, status = (
                                                    calculate_wall_slope_and_error(
                                                        wall_points
                                                    )
                                                )
                                                if slope is not None:
                                                    # This is the main output for the test
                                                    print(
                                                        f"Rotation #{rotation_count}: Angle Error = {angle_error:.2f}°"
                                                    )
                                                    print(servo_angle)
                                                    servo_angle = MID_SERVO - (
                                                        angle_error * GYROPG
                                                    )

                                                else:
                                                    print(
                                                        f"Rotation #{rotation_count}: Could not calculate angle."
                                                    )
                                            else:
                                                print("inssuficient again")

                                    current_scan_points = []

                                if 0 < dist < 1000:
                                    angle_rad = math.radians(angle)
                                    x = dist * math.cos(angle_rad) / 10.0
                                    y = dist * math.sin(angle_rad) / 10.0

                                    if y < 0 and y > -15 and x < 30 and x > 10:
                                        print(
                                            f"RIGHT SIDE DETECTION: x={x:.1f}, y={y:.1f}, right_reading updated from {right_reading} to {x}"
                                        )
                                        right_reading = False
                                        servo_angle = MID_SERVO

                                    current_scan_points.append((x, y))

                                previous_angle = angle

                        buffer = buffer[idx + PACKET_LEN :]
                    if (servo_angle) > MID_SERVO + MAX_TURN_DEGREE:
                        servo_angle = MID_SERVO + MAX_TURN_DEGREE
                    elif (servo_angle) < MID_SERVO - MAX_TURN_DEGREE:
                        servo_angle = MID_SERVO - MAX_TURN_DEGREE
                    time.sleep(0.02)

                    board.pwm_servo_set_position(0.02, [[1, pwm(servo_angle)]])

                    time.sleep(0.02)

                except Exception as e:
                    print(f"An error occurred: {e}")

                except KeyboardInterrupt:
                    print("\nProgram ended.")
                    board.pwm_servo_set_position(0.04, [[2, 1500]])
                    board.pwm_servo_set_position(0.04, [[1, pwm(MID_SERVO)]])
                    ser.close()
                    break
            servo_angle = MID_SERVO
            while not passed_wall:

                try:
                    bytes_to_read = ser.in_waiting
                    if bytes_to_read > 0:
                        buffer += ser.read(bytes_to_read)

                    while len(buffer) >= PACKET_LEN:
                        idx = find_packet_start(buffer)
                        if idx == -1:
                            buffer = buffer[-(PACKET_LEN - 1) :]
                            break
                        if len(buffer) - idx < PACKET_LEN:
                            buffer = buffer[idx:]
                            break

                        packet = buffer[idx : idx + PACKET_LEN]
                        parsed = parse_packet(packet)

                        if parsed:
                            angles = interpolate_angles(
                                parsed["start_angle"], parsed["end_angle"], 12
                            )
                            for (dist, conf), angle in zip(parsed["points"], angles):
                                if detect_angle_reset(angle, previous_angle):
                                    if len(current_scan_points) > 50:
                                        rotation_count += 1
                                        wall_points = filter_wall_points(
                                            current_scan_points,
                                            x_min=10,
                                            x_max=100,
                                            y_min=-9,
                                            y_max=0,
                                        )

                                    current_scan_points = []

                                if 0 < dist < 1000:
                                    angle_rad = math.radians(angle)
                                    x = dist * math.cos(angle_rad) / 10.0
                                    y = dist * math.sin(angle_rad) / 10.0

                                    if y < -21 and y > -23 and x < 24 and x > 10:
                                        print(
                                            f"RIGHT SIDE DETECTION: x={x:.1f}, y={y:.1f}, right_reading updated from {right_reading} to {x}"
                                        )
                                        passed_wall = True
                                        print("pass")
                                    current_scan_points.append((x, y))

                                previous_angle = angle

                        buffer = buffer[idx + PACKET_LEN :]
                    if (servo_angle) > MID_SERVO + MAX_TURN_DEGREE:
                        servo_angle = MID_SERVO + MAX_TURN_DEGREE
                    elif (servo_angle) < MID_SERVO - MAX_TURN_DEGREE:
                        servo_angle = MID_SERVO - MAX_TURN_DEGREE

                    time.sleep(0.02)

                    board.pwm_servo_set_position(0.04, [[1, pwm(servo_angle)]])
                    time.sleep(0.02)

                except Exception as e:
                    print(f"An error occurred: {e}")

                except KeyboardInterrupt:
                    print("\nProgram ended.")
                    board.pwm_servo_set_position(0.04, [[2, 1500]])
                    board.pwm_servo_set_position(0.04, [[1, pwm(MID_SERVO)]])
                    ser.close()
                    break
            time.sleep(0.2)
            at_parking_lot = True
            board.pwm_servo_set_position(0.04, [[1, pwm(MID_SERVO)]])
            board.pwm_servo_set_position(0.04, [[2, 1500]])
            time.sleep(3)

            board.pwm_servo_set_position(0.04, [[2, DC_SPEED + 10]])

            servo_angle = 0
            board.pwm_servo_set_position(0.04, [[1, pwm(servo_angle)]])
            time.sleep(2)
            board.pwm_servo_set_position(0.04, [[1, pwm(MID_SERVO)]])
            time.sleep(0.5)

            board.pwm_servo_set_position(0.04, [[2, 1595]])

            current_scan_points = []
            previous_angle = None
            rotation_count = 0

            ser.reset_input_buffer()
            buffer = b""
            while running:

                try:
                    bytes_to_read = ser.in_waiting
                    if bytes_to_read > 0:
                        buffer += ser.read(bytes_to_read)

                    while len(buffer) >= PACKET_LEN:
                        idx = find_packet_start(buffer)
                        if idx == -1:
                            buffer = buffer[-(PACKET_LEN - 1) :]
                            break
                        if len(buffer) - idx < PACKET_LEN:
                            buffer = buffer[idx:]
                            break

                        packet = buffer[idx : idx + PACKET_LEN]
                        parsed = parse_packet(packet)

                        if parsed:
                            angles = interpolate_angles(
                                parsed["start_angle"], parsed["end_angle"], 12
                            )
                            for (dist, conf), angle in zip(parsed["points"], angles):
                                if detect_angle_reset(angle, previous_angle):
                                    if len(current_scan_points) > 50:
                                        rotation_count += 1
                                        wall_points = filter_wall_points(
                                            current_scan_points,
                                            x_min=-15,
                                            x_max=5,
                                            y_min=-70,
                                            y_max=-10,
                                        )

                                        if len(wall_points) >= 8:
                                            slope, angle_error, status = (
                                                calculate_wall_slope_and_error(
                                                    wall_points
                                                )
                                            )

                                            angle_error = math.degrees(
                                                math.atan(1 / slope)
                                            )

                                            if slope is not None:
                                                # This is the main output for the test
                                                print(
                                                    f"Rotation #{rotation_count}: Angle Error = {angle_error:.2f}°"
                                                )
                                                print(servo_angle)
                                                servo_angle = MID_SERVO - (
                                                    angle_error * GYROPG
                                                )

                                            else:
                                                print(
                                                    f"Rotation #{rotation_count}: Could not calculate angle."
                                                )
                                        else:
                                            print(
                                                f"Rotation #{rotation_count}: Insufficient wall points."
                                            )

                                    current_scan_points = []

                                if 0 < dist < 1000:
                                    angle_rad = math.radians(angle)
                                    x = dist * math.cos(angle_rad) / 10.0
                                    y = dist * math.sin(angle_rad) / 10.0

                                    if y < -65 and x < 2 and x > -5:
                                        print(f"FRONT DETECTION: x={x:.1f}, y={y:.1f}")
                                        running = False

                                    current_scan_points.append((x, y))

                                previous_angle = angle

                        buffer = buffer[idx + PACKET_LEN :]
                    if (servo_angle) > MID_SERVO + MAX_TURN_DEGREE:
                        servo_angle = MID_SERVO + MAX_TURN_DEGREE
                    elif (servo_angle) < MID_SERVO - MAX_TURN_DEGREE:
                        servo_angle = MID_SERVO - MAX_TURN_DEGREE

                    time.sleep(0.02)

                    board.pwm_servo_set_position(0.02, [[1, pwm(servo_angle)]])
                    time.sleep(0.02)

                except Exception as e:
                    print(f"An error occurred: {e}")

                except KeyboardInterrupt:
                    print("\nProgram ended.")
                    board.pwm_servo_set_position(0.04, [[2, 1500]])
                    board.pwm_servo_set_position(0.04, [[1, pwm(MID_SERVO)]])
                    ser.close()
                    break

            servo_angle = MID_SERVO
            board.pwm_servo_set_position(0.04, [[1, pwm(servo_angle)]])
            board.pwm_servo_set_position(0.04, [[2, 1600]])

            time.sleep(0.1)

            servo_angle = MID_SERVO + 30
            board.pwm_servo_set_position(0.04, [[1, pwm(servo_angle)]])
            board.pwm_servo_set_position(0.04, [[2, 1600]])

            time.sleep(2.15)

            stopMove = True
    """
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    """
    if parking_right:
        print("parking right")
        if clear == False:

            servo_angle = MID_SERVO - (MAX_TURN_DEGREE)
            board.pwm_servo_set_position(0.04, [[2, DC_SPEED]])
            board.pwm_servo_set_position(0.04, [[1, pwm(servo_angle)]])
            time.sleep(0.4)

            servo_angle = MID_SERVO
            board.pwm_servo_set_position(0.04, [[2, DC_SPEED]])
            board.pwm_servo_set_position(0.04, [[1, pwm(servo_angle)]])
            time.sleep(0.4)

            clear = True
            board.pwm_servo_set_position(0.04, [[2, DC_SPEED]])

        current_scan_points = []
        previous_angle = None
        rotation_count = 0

        ser.reset_input_buffer()
        buffer = b""
        board.pwm_servo_set_position(0.04, [[2, DC_SPEED + 25]])

        while not close_to_wall:

            try:
                bytes_to_read = ser.in_waiting
                if bytes_to_read > 0:
                    buffer += ser.read(bytes_to_read)

                while len(buffer) >= PACKET_LEN:
                    idx = find_packet_start(buffer)
                    if idx == -1:
                        buffer = buffer[-(PACKET_LEN - 1) :]
                        break
                    if len(buffer) - idx < PACKET_LEN:
                        buffer = buffer[idx:]
                        break

                    packet = buffer[idx : idx + PACKET_LEN]
                    parsed = parse_packet(packet)

                    if parsed:
                        angles = interpolate_angles(
                            parsed["start_angle"], parsed["end_angle"], 12
                        )
                        for (dist, conf), angle in zip(parsed["points"], angles):
                            if detect_angle_reset(angle, previous_angle):
                                if len(current_scan_points) > 50:
                                    rotation_count += 1

                                    wall_points = filter_wall_points(
                                        current_scan_points,
                                        x_min=-70,
                                        x_max=-1,
                                        y_min=-8,
                                        y_max=-1,
                                    )

                                    if len(wall_points) >= 8:
                                        slope, angle_error, status = (
                                            calculate_wall_slope_and_error(wall_points)
                                        )

                                        if slope is not None:
                                            # This is the main output for the test
                                            print(
                                                f"Rotation #{rotation_count}: Angle Error = {angle_error:.2f}°"
                                            )
                                            print(slope)
                                            servo_angle = MID_SERVO - (
                                                angle_error * GYROPG
                                            )

                                        else:
                                            print(
                                                f"Rotation #{rotation_count}: Could not calculate angle."
                                            )
                                    else:
                                        print(
                                            f"Rotation #{rotation_count}: Insufficient wall points."
                                        )

                                current_scan_points = []

                            if 0 < dist < 1000:
                                angle_rad = math.radians(angle)
                                x = dist * math.cos(angle_rad) / 10.0
                                y = dist * math.sin(angle_rad) / 10.0

                                if y > -11 and y < -9 and x > -3 and x < 3:
                                    close_to_wall = True

                                current_scan_points.append((x, y))

                            previous_angle = angle

                    buffer = buffer[idx + PACKET_LEN :]
                if (servo_angle) > MID_SERVO + MAX_TURN_DEGREE:
                    servo_angle = MID_SERVO + MAX_TURN_DEGREE
                elif (servo_angle) < MID_SERVO - MAX_TURN_DEGREE:
                    servo_angle = MID_SERVO - MAX_TURN_DEGREE
                time.sleep(0.02)
                board.pwm_servo_set_position(0.02, [[1, pwm(servo_angle)]])
                time.sleep(0.02)

            except Exception as e:
                print(f"An error occurred: {e}")

            except KeyboardInterrupt:
                print("\nProgram ended.")
                board.pwm_servo_set_position(0.04, [[2, 1500]])
                board.pwm_servo_set_position(0.04, [[1, pwm(MID_SERVO)]])
                ser.close()
                break

        if not turn_to_parking_section:
            print("HELLOOOOOOOO??????")
            board.pwm_servo_set_position(0.04, [[1, pwm(MID_SERVO)]])

            board.pwm_servo_set_position(0.04, [[2, 1500]])

            board.pwm_servo_set_position(0.04, [[1, pwm(servo_angle)]])
            time.sleep(0.5)
            servo_angle = 30
            board.pwm_servo_set_position(0.04, [[2, 1620]])

            board.pwm_servo_set_position(0.04, [[1, pwm(servo_angle)]])
            time.sleep(1.8)
            servo_angle = MID_SERVO
            board.pwm_servo_set_position(0.04, [[1, pwm(servo_angle)]])

            board.pwm_servo_set_position(0.04, [[2, 1500]])

            time.sleep(0.5)

            turn_to_parking_section = True

        else:

            print("AINFSAF")

            current_scan_points = []
            previous_angle = None
            rotation_count = 0

            ser.reset_input_buffer()
            buffer = b""
            time.sleep(0.5)
            board.pwm_servo_set_position(0.04, [[2, DC_SPEED + 15]])

            while right_reading:

                try:
                    bytes_to_read = ser.in_waiting
                    if bytes_to_read > 0:
                        buffer += ser.read(bytes_to_read)

                    while len(buffer) >= PACKET_LEN:
                        idx = find_packet_start(buffer)
                        if idx == -1:
                            buffer = buffer[-(PACKET_LEN - 1) :]
                            break
                        if len(buffer) - idx < PACKET_LEN:
                            buffer = buffer[idx:]
                            break

                        packet = buffer[idx : idx + PACKET_LEN]
                        parsed = parse_packet(packet)

                        if parsed:
                            angles = interpolate_angles(
                                parsed["start_angle"], parsed["end_angle"], 12
                            )
                            for (dist, conf), angle in zip(parsed["points"], angles):
                                if detect_angle_reset(angle, previous_angle):
                                    if len(current_scan_points) > 50:
                                        rotation_count += 1
                                        wall_points = filter_wall_points(
                                            current_scan_points,
                                            x_min=-100,
                                            x_max=-20,
                                            y_min=-7,
                                            y_max=0,
                                        )

                                        if len(wall_points) >= 8:
                                            slope, angle_error, status = (
                                                calculate_wall_slope_and_error(
                                                    wall_points
                                                )
                                            )
                                            if slope is not None:
                                                # This is the main output for the test
                                                print(
                                                    f"Rotation #{rotation_count}: Angle Error = {angle_error:.2f}°"
                                                )
                                                print(servo_angle)
                                                servo_angle = MID_SERVO - (
                                                    angle_error * GYROPG
                                                )

                                            else:
                                                print(
                                                    f"Rotation #{rotation_count}: Could not calculate angle."
                                                )
                                        else:
                                            print(
                                                f"Rotation #{rotation_count}: Insufficient wall points."
                                            )

                                    current_scan_points = []

                                if 0 < dist < 1000:
                                    angle_rad = math.radians(angle)
                                    x = dist * math.cos(angle_rad) / 10.0
                                    y = dist * math.sin(angle_rad) / 10.0

                                    if y < 0 and y > -8 and x > -30 and x < -10:
                                        print(
                                            f"LEFT SIDE DETECTION: x={x:.1f}, y={y:.1f}, right_reading updated from {right_reading} to {x}"
                                        )
                                        right_reading = False

                                    current_scan_points.append((x, y))

                                previous_angle = angle

                        buffer = buffer[idx + PACKET_LEN :]
                    if (servo_angle) > MID_SERVO + MAX_TURN_DEGREE:
                        servo_angle = MID_SERVO + MAX_TURN_DEGREE
                    elif (servo_angle) < MID_SERVO - MAX_TURN_DEGREE:
                        servo_angle = MID_SERVO - MAX_TURN_DEGREE
                    time.sleep(0.02)
                    board.pwm_servo_set_position(0.04, [[1, pwm(servo_angle)]])
                    time.sleep(0.02)

                except Exception as e:
                    print(f"An error occurred: {e}")

                except KeyboardInterrupt:
                    print("\nProgram ended.")
                    board.pwm_servo_set_position(0.04, [[2, 1500]])
                    board.pwm_servo_set_position(0.04, [[1, pwm(MID_SERVO)]])
                    ser.close()
                    break
            servo_angle = MID_SERVO

            while not passed_wall:

                try:
                    bytes_to_read = ser.in_waiting
                    if bytes_to_read > 0:
                        buffer += ser.read(bytes_to_read)

                    while len(buffer) >= PACKET_LEN:
                        idx = find_packet_start(buffer)
                        if idx == -1:
                            buffer = buffer[-(PACKET_LEN - 1) :]
                            break
                        if len(buffer) - idx < PACKET_LEN:
                            buffer = buffer[idx:]
                            break

                        packet = buffer[idx : idx + PACKET_LEN]
                        parsed = parse_packet(packet)

                        if parsed:
                            angles = interpolate_angles(
                                parsed["start_angle"], parsed["end_angle"], 12
                            )
                            for (dist, conf), angle in zip(parsed["points"], angles):
                                if detect_angle_reset(angle, previous_angle):
                                    if len(current_scan_points) > 50:
                                        rotation_count += 1
                                        wall_points = filter_wall_points(
                                            current_scan_points,
                                            x_min=10,
                                            x_max=100,
                                            y_min=-9,
                                            y_max=0,
                                        )

                                    current_scan_points = []

                                if 0 < dist < 1000:
                                    angle_rad = math.radians(angle)
                                    x = dist * math.cos(angle_rad) / 10.0
                                    y = dist * math.sin(angle_rad) / 10.0

                                    if y < -11 and y > -13 and x > -22 and x < -10:
                                        print(
                                            f"RIGHT SIDE DETECTION: x={x:.1f}, y={y:.1f}, right_reading updated from {right_reading} to {x}"
                                        )
                                        passed_wall = True
                                        print("pass")
                                    current_scan_points.append((x, y))

                                previous_angle = angle

                        buffer = buffer[idx + PACKET_LEN :]
                    if (servo_angle) > MID_SERVO + MAX_TURN_DEGREE:
                        servo_angle = MID_SERVO + MAX_TURN_DEGREE
                    elif (servo_angle) < MID_SERVO - MAX_TURN_DEGREE:
                        servo_angle = MID_SERVO - MAX_TURN_DEGREE

                    time.sleep(0.02)

                    board.pwm_servo_set_position(0.04, [[1, pwm(servo_angle)]])
                    time.sleep(0.02)

                except Exception as e:
                    print(f"An error occurred: {e}")

                except KeyboardInterrupt:
                    print("\nProgram ended.")
                    board.pwm_servo_set_position(0.04, [[2, 1500]])
                    board.pwm_servo_set_position(0.04, [[1, pwm(MID_SERVO)]])
                    ser.close()
                    break

            at_parking_lot = True
            board.pwm_servo_set_position(0.04, [[2, 1500]])
            time.sleep(3)

            board.pwm_servo_set_position(0.04, [[2, DC_SPEED + 25]])
            servo_angle = MID_SERVO + MAX_TURN_DEGREE
            board.pwm_servo_set_position(0.04, [[1, pwm(servo_angle)]])
            time.sleep(2.2)

            servo_angle = MID_SERVO
            board.pwm_servo_set_position(0.04, [[1, pwm(servo_angle)]])
            time.sleep(0.5)

            board.pwm_servo_set_position(0.04, [[2, 1595]])

            current_scan_points = []
            previous_angle = None
            rotation_count = 0

            ser.reset_input_buffer()
            buffer = b""
            while running:

                try:
                    bytes_to_read = ser.in_waiting
                    if bytes_to_read > 0:
                        buffer += ser.read(bytes_to_read)

                    while len(buffer) >= PACKET_LEN:
                        idx = find_packet_start(buffer)
                        if idx == -1:
                            buffer = buffer[-(PACKET_LEN - 1) :]
                            break
                        if len(buffer) - idx < PACKET_LEN:
                            buffer = buffer[idx:]
                            break

                        packet = buffer[idx : idx + PACKET_LEN]
                        parsed = parse_packet(packet)

                        if parsed:
                            angles = interpolate_angles(
                                parsed["start_angle"], parsed["end_angle"], 12
                            )
                            for (dist, conf), angle in zip(parsed["points"], angles):
                                if detect_angle_reset(angle, previous_angle):
                                    if len(current_scan_points) > 50:
                                        rotation_count += 1
                                        wall_points = filter_wall_points(
                                            current_scan_points,
                                            x_min=-10,
                                            x_max=0,
                                            y_min=-70,
                                            y_max=-10,
                                        )

                                        if len(wall_points) >= 8:
                                            slope, angle_error, status = (
                                                calculate_wall_slope_and_error(
                                                    wall_points
                                                )
                                            )

                                            angle_error = math.degrees(
                                                math.atan(1 / slope)
                                            )

                                            if slope is not None:
                                                # This is the main output for the test
                                                print(
                                                    f"Rotation #{rotation_count}: Angle Error = {angle_error:.2f}°"
                                                )
                                                print(servo_angle)
                                                servo_angle = MID_SERVO - (
                                                    angle_error * GYROPG
                                                )

                                            else:
                                                print(
                                                    f"Rotation #{rotation_count}: Could not calculate angle."
                                                )
                                        else:
                                            print(
                                                f"Rotation #{rotation_count}: Insufficient wall points."
                                            )

                                    current_scan_points = []

                                if 0 < dist < 1000:
                                    angle_rad = math.radians(angle)
                                    x = dist * math.cos(angle_rad) / 10.0
                                    y = dist * math.sin(angle_rad) / 10.0

                                    if y < -65 and x < 5 and x > -5:
                                        print(f"FRONT DETECTION: x={x:.1f}, y={y:.1f}")
                                        running = False

                                    current_scan_points.append((x, y))

                                previous_angle = angle

                        buffer = buffer[idx + PACKET_LEN :]
                    if (servo_angle) > MID_SERVO + MAX_TURN_DEGREE:
                        servo_angle = MID_SERVO + MAX_TURN_DEGREE
                    elif (servo_angle) < MID_SERVO - MAX_TURN_DEGREE:
                        servo_angle = MID_SERVO - MAX_TURN_DEGREE
                    time.sleep(0.02)
                    board.pwm_servo_set_position(0.04, [[1, pwm(servo_angle)]])
                    time.sleep(0.02)

                except Exception as e:
                    print(f"An error occurred: {e}")

                except KeyboardInterrupt:
                    print("\nProgram ended.")
                    board.pwm_servo_set_position(0.04, [[2, 1500]])
                    board.pwm_servo_set_position(0.04, [[1, pwm(MID_SERVO)]])
                    ser.close()
                    break

            servo_angle = MID_SERVO
            board.pwm_servo_set_position(0.04, [[1, pwm(servo_angle)]])
            board.pwm_servo_set_position(0.04, [[2, 1600]])

            time.sleep(0.1)

            servo_angle = MID_SERVO + 30
            board.pwm_servo_set_position(0.04, [[1, pwm(servo_angle)]])
            board.pwm_servo_set_position(0.04, [[2, 1600]])

            time.sleep(2.2)

            stopMove = True

    if servo_angle > MID_SERVO:
        servo_angle = MID_SERVO + (servo_angle - MID_SERVO) * 0.9

    if (servo_angle) > MID_SERVO + MAX_TURN_DEGREE:
        servo_angle = MID_SERVO + MAX_TURN_DEGREE
    elif (servo_angle) < MID_SERVO - MAX_TURN_DEGREE:
        servo_angle = MID_SERVO - MAX_TURN_DEGREE

    """
    if front_area_black >= 4000 and target == None:




        if track_dir == "left":
            if target == None:
                servo_angle = MID_SERVO - MAX_TURN_DEGREE
        elif track_dir == "right":

            if target == None:
                servo_angle = MID_SERVO + MAX_TURN_DEGREE



    """

    print(
        str(turn_counter)
        + " "
        + str(turn_dir)
        + " "
        + str(target)
        + " "
        + str(servo_angle)
        + " "
        + str(track_dir)
        + " "
        + str(left_area)
        + " "
        + str(right_area)
        + " "
        + str(closest_pillar_distance)
        + " "
        + str(front_area_black)
        + " "
        + str(closest_pillar_x)
        + " "
        + str(orange_line_area)
    )

    if (
        not exit_parking_lot_green
        or exit_parking_lot_red
        and not (parking_right or parking_right)
    ):
        pw = pwm(servo_angle)

        board.pwm_servo_set_position(0.04, [[1, pw]])

    if target != None:
        image = cv2.line(im, (target, 0), (target, 520), (255, 255, 0), 1)

    drawROI(ROI_LEFT)
    drawROI(ROI_RIGHT)
    drawROI(ROI_MIDDLE)
    drawROI(ROI_LINE)
    drawROI(ROI_FRONT)
    fps_counter += 1
    current_time = time.time()
    if current_time - fps_start_time >= 1.0:
        fps = fps_counter / (current_time - fps_start_time)
        fps_counter = 0
        fps_start_time = current_time

    cv2.putText(
        im, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2
    )

    cv2.imshow("Camera", im)

    if cv2.waitKey(1) == ord("q") or action_counter >= ACTIONS_TO_STRAIGHT or stopMove:
        print("stopped")
        board.pwm_servo_set_position(0.04, [[2, 1500]])
        print(stopMove)
        stop()
        break
stop()
