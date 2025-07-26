import cv2
import numpy as np
import time
import ros_robot_controller_sdk as rrc
import math

from picamera2 import Picamera2
import RPi.GPIO as GPIO

# Imports for LD19 LIDAR parsing
import serial
import struct

board = rrc.Board()

MID_SERVO = 65
MAX_TURN_DEGREE = 26
DC_SPEED = 1380
PILLAR_SIZE = 400

PD = 0
PG = 0.0017

ROI_LEFT = [0, 230, 100, 450]
ROI_RIGHT = [540, 230, 640, 450]
ROI_LINE = [270, 390, 370, 410]
ROI_MIDDLE = [0, 200, 640, 400]

ROI_FRONT = [260, 90, 380, 150]


LOWER_BLACK_THRESHOLD = np.array([0, 0, 0])
UPPER_BLACK_THRESHOLD = np.array([180, 255, 70])

LOWER_RED_THRESHOLD1 = np.array([0, 154, 70])
UPPER_RED_THRESHOLD1 = np.array([4, 255, 255])
LOWER_RED_THRESHOLD2 = np.array([176, 180, 70])
UPPER_RED_THRESHOLD2 = np.array([180, 255, 255])

LOWER_GREEN_THRESHOLD = np.array([65, 85, 45])
UPPER_GREEN_THRESHOLD = np.array([89, 255, 185])

LOWER_ORANGE_THRESHOLD1 = np.array([180, 80, 80])
UPPER_ORANGE_THRESHOLD1 = np.array([180, 255, 255])
LOWER_ORANGE_THRESHOLD2 = np.array([0, 80, 100])
UPPER_ORANGE_THRESHOLD2 = np.array([20, 255, 255])

LOWER_BLUE_THRESHOLD = np.array([108, 45, 75])
UPPER_BLUE_THRESHOLD = np.array([130, 255, 170])

LOWER_MAGENTA_THRESHOLD1 = np.array([0, 100, 50])
UPPER_MAGENTA_THRESHOLD1 = np.array([0, 215, 255])

LOWER_MAGENTA_THRESHOLD2 = np.array([150, 150, 70])
UPPER_MAGENTA_THRESHOLD2 = np.array([172, 255, 255])

WIDTH = 640
HEIGHT = 480
POINTS = [(115, 200), (525, 200), (640, 370), (0, 370)]

MAX_TURNS = 1
ACTIONS_TO_STRAIGHT = 120
WALL_THRESHOLD = 400
NO_WALL_THRESHOLD = 200
TURN_ITER_LIMIT = 30
LINE_THRESHOLD = 100
action_counter = 0
turn_counter = 0
turn_length_counter = 0

right_reading = None
parking = False
start = False
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

red_target = 105
green_target = 535

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
lidar_buffer = bytearray()


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
    time.sleep(0.02)
    board.pwm_servo_set_position(0.1, [[2, 1500]])
    board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
    cv2.destroyAllWindows()


def findMaxContour(contours):
    max_area = 0
    for i in range(len(contours)):
        area = cv2.contourArea(contours[i])
        max_area = max(area, max_area)
    return max_area


board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
board.pwm_servo_set_position(0.1, [[2, 1500]])
time.sleep(0.5)

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
    img_thresh_magenta = cv2.bitwise_or(
        cv2.inRange(img_hsv, LOWER_MAGENTA_THRESHOLD1, UPPER_MAGENTA_THRESHOLD1),
        cv2.inRange(img_hsv, LOWER_MAGENTA_THRESHOLD2, UPPER_MAGENTA_THRESHOLD2),
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

    left_contours_magenta, _ = cv2.findContours(
        img_thresh_magenta[
            ROI_LEFT[1] : ROI_LEFT[3],
            ROI_LEFT[0] : ROI_LEFT[2],
        ],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )

    right_contours_magenta, _ = cv2.findContours(
        img_thresh_magenta[
            ROI_RIGHT[1] : ROI_RIGHT[3],
            ROI_RIGHT[0] : ROI_RIGHT[2],
        ],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )

    center_contours_magenta, _ = cv2.findContours(
        img_thresh_magenta[
            ROI_FRONT[1] : ROI_FRONT[3],
            ROI_FRONT[0] : ROI_FRONT[2],
        ],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )

    middle_contours_magenta, _ = cv2.findContours(
        img_thresh_magenta[
            ROI_MIDDLE[1] : ROI_MIDDLE[3], ROI_MIDDLE[0] : ROI_MIDDLE[2]
        ],
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

    middle_contours_black, hierarchy = cv2.findContours(
        img_thresh_black[ROI_LINE[1] : ROI_LINE[3], ROI_LINE[0] : ROI_LINE[2]],
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
    middle_area_black = 0
    green_pillar_area = 0
    red_pillar_area = 0
    blue_line_area = 0
    orange_line_area = 0
    magenta_area_middle = 0
    magenta_area_left = 0
    magenta_area_right = 0

    contours_coloured = [
        (contours_red, (0, 0, 255), 1, ROI_MIDDLE),
        (contours_green, (0, 255, 0), 1, ROI_MIDDLE),
        (contours, (0, 255, 255), 1, (0, 0)),
        (contours_orange, (52, 140, 235), 0, ROI_LINE),
        (contours_blue, (235, 67, 52), 0, ROI_LINE),
        (left_contours_magenta, (255, 0, 255), 1, ROI_LEFT),
        (right_contours_magenta, (255, 0, 255), 1, ROI_RIGHT),
        (middle_contours_magenta, (255, 0, 255), 1, ROI_MIDDLE),
    ]

    left_area_black = findMaxContour(left_contours_black)
    right_area_black = findMaxContour(right_contours_black)
    middle_area_black = findMaxContour(middle_contours_black)
    front_area_black = findMaxContour(front_contours_black)
    left_area = left_area_black
    right_area = right_area_black

    for i in range(len(contours_orange)):
        cnt = contours_orange[i]
        orange_line_area = max(cv2.contourArea(cnt), orange_line_area)
        cnt[:, :, 0] += ROI_LINE[0]
        cnt[:, :, 1] += ROI_LINE[1]
        cv2.drawContours(im, contours_orange, i, (255, 255, 0), 1)

    for i in range(len(contours_blue)):
        cnt = contours_blue[i]
        blue_line_area = max(cv2.contourArea(cnt), blue_line_area)
        cnt[:, :, 0] += ROI_LINE[0]
        cnt[:, :, 1] += ROI_LINE[1]

        cv2.drawContours(im, contours_blue, i, (255, 255, 0), 1)

    for i in range(len(middle_contours_magenta)):
        cnt = middle_contours_magenta[i]
        magenta_area_middle = max(cv2.contourArea(cnt), magenta_area_middle)
        cnt[:, :, 0] += ROI_MIDDLE[0]
        cnt[:, :, 1] += ROI_MIDDLE[1]

    for i in range(len(left_contours_magenta)):
        cnt = left_contours_magenta[i]
        magenta_area_left = max(cv2.contourArea(cnt), magenta_area_left)
        cnt[:, :, 0] += ROI_LEFT[0]
        cnt[:, :, 1] += ROI_LEFT[1]

    for i in range(len(right_contours_magenta)):
        cnt = right_contours_magenta[i]
        magenta_area_right = max(cv2.contourArea(cnt), magenta_area_right)
        cnt[:, :, 0] += ROI_RIGHT[0]
        cnt[:, :, 1] += ROI_RIGHT[1]

    for c in contours_coloured:
        cont = c[0]
        colour = c[1]
        ROIROI = c[3]
        for i, cnt in enumerate(cont):
            area = cv2.contourArea(cnt)
            if area > 100:
                cnt[:, :, 0] += ROIROI[0]
                cnt[:, :, 1] += ROIROI[1]
                cv2.drawContours(im, [cnt], -1, colour, 2)
    if start:
        print("fajf " + str(right_area) + " " + str(left_area))
        if right_area > 1500:
            track_dir = "left"
            exit_parking_lot_left = True
            start = False
        elif left_area > 1500:
            track_dir = "right"

            exit_parking_lot_right = True
            start = False

    if exit_parking_lot_left:
        pw = pwm(MID_SERVO - 35)
        print("hello left i am turning " + str(MID_SERVO - 30))
        board.pwm_servo_set_position(0.1, [[1, pw]])
        time.sleep(0.1)
        board.pwm_servo_set_position(0.1, [[2, 1400]])

        time.sleep(0.4)

        turn_dir = None
        closest_pillar_distance = 10000
        closest_pillar_colour = None
        closest_pillar_area = -1
        if not a:
            a = True
            continue

        for i in contours_green:
            area = cv2.contourArea(i)

            if area > PILLAR_SIZE:
                approx = cv2.approxPolyDP(i, 0.01 * cv2.arcLength(i, True), True)
                x, y, w, h = cv2.boundingRect(approx)

                pillar_distance = math.dist([x + w // 2, y], [320, 480])

                image = cv2.line(im, (x, y), (x + w, y), (0, 255, 255), 1)
                image = cv2.line(im, (x, y), ((x, h + y)), (0, 255, 255), 1)
                image = cv2.line(im, (x + w, y), (x + w, y + h), (0, 255, 255), 1)
                image = cv2.line(im, (x, y + h), (x + w, y + h), (0, 255, 255), 1)
                cv2.circle(im, (int(x + (w / 2)), y), 5, (255, 255, 0), 1, -1)

                if pillar_distance < closest_pillar_distance:

                    closest_pillar_distance = pillar_distance
                    closest_pillar_colour = "green"
                    closest_pillar_area = h * w

        print("aaaa" + str(closest_pillar_area) + " " + str(closest_pillar_distance))
        exit_parking_lot_left = False
        if (
            closest_pillar_colour == "green"
            and closest_pillar_area > 1500
            and closest_pillar_distance < 480
        ):
            pw = pwm(MID_SERVO - (MAX_TURN_DEGREE))
            board.pwm_servo_set_position(0.1, [[2, 1390]])
            board.pwm_servo_set_position(0.1, [[1, pw]])
            time.sleep(0.9)
            exit_parking_lot_green = True

        else:
            board.pwm_servo_set_position(0.1, [[2, 1500]])

            time.sleep(0.2)

    if exit_parking_lot_right:
        pw = pwm(MID_SERVO + 28)
        print("hello i am turning " + str(MID_SERVO + 30))
        board.pwm_servo_set_position(0.1, [[1, pw]])
        time.sleep(0.1)

        board.pwm_servo_set_position(0.1, [[2, 1400]])
        time.sleep(0.3)
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

    OBSTACLEPG = 0.0012
    OBSTACLEPD = 0.0022
    YAXISPG = 0.006

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

            if pillar_distance < 600:
                image = cv2.line(im, (x, y), (x + w, y), (0, 255, 255), 1)
                image = cv2.line(im, (x, y), ((x, h + y)), (0, 255, 255), 1)
                image = cv2.line(im, (x + w, y), (x + w, y + h), (0, 255, 255), 1)
                image = cv2.line(im, (x, y + h), (x + w, y + h), (0, 255, 255), 1)
                cv2.circle(im, (int(x + (w / 2)), y), 5, (255, 255, 0), 1, -1)

                num_pillars_r += 1

                if pillar_distance < closest_pillar_distance:
                    if y + h > 400 and x + (w // 2) < red_target:
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

            if pillar_distance < 600:
                image = cv2.line(im, (x, y), (x + w, y), (0, 255, 255), 1)
                image = cv2.line(im, (x, y), ((x, h + y)), (0, 255, 255), 1)
                image = cv2.line(im, (x + w, y), (x + w, y + h), (0, 255, 255), 1)
                image = cv2.line(im, (x, y + h), (x + w, y + h), (0, 255, 255), 1)
                cv2.circle(im, (int(x + (w / 2)), y), 5, (255, 255, 0), 1, -1)

                num_pillars_g += 1

                if pillar_distance < closest_pillar_distance:
                    if y + h > 400 and (x + (w // 2) > green_target):
                        prevPillar = "green"
                        pass
                    elif y + h < 125:
                        pass
                    else:
                        closest_pillar_distance = pillar_distance
                        closest_pillar_colour = "green"
                        closest_pillar_x = x + w // 2
                        closest_pillar_y = y
                        closest_pillar_area = h * w

    if turn_counter >= MAX_TURNS:
        print("_----------" + str(magenta_area_right))
        if magenta_area_right > 1000:
            parking = True

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

    if target != None and turn_counter < MAX_TURNS:
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
                closest_pillar_area > 6000
                and (closest_pillar_x) < 390
                and closest_pillar_distance < 300
                and not exit_parking_lot
            ):
                servo_angle = MID_SERVO  # set the servo to straight
                pw = pwm(servo_angle)

                board.pwm_servo_set_position(0.1, [[2, 1500]])
                board.pwm_servo_set_position(0.1, [[1, pw]])

                board.pwm_servo_set_position(0.1, [[2, 1620]])
                time.sleep(0.8)

        if closest_pillar_colour == "red":
            servo_angle += closest_pillar_y * YAXISPG

            if (
                # if the pillar is too close to the robot and not on the right side, reverse the robot
                closest_pillar_area > 6000
                and (closest_pillar_x) > 250
                and closest_pillar_distance < 300
                and not exit_parking_lot
            ):
                servo_angle = MID_SERVO  # set the servo to straight
                pw = pwm(servo_angle)

                board.pwm_servo_set_position(0.1, [[2, 1500]])
                board.pwm_servo_set_position(0.1, [[1, pw]])

                board.pwm_servo_set_position(0.1, [[2, 1620]])
                time.sleep(0.8)

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

        if turn_dir == "right":
            servo_angle = MID_SERVO + (MAX_TURN_DEGREE * 0.6)
        elif turn_dir == "left":
            servo_angle = MID_SERVO - (MAX_TURN_DEGREE * 0.6)
        elif turn_dir == None:
            current_difference = left_area - right_area

            servo_angle = MID_SERVO + (
                current_difference * PG + (current_difference - last_difference) * PD
            )

    last_difference = current_difference
    if turn_dir == None or target != None:
        if left_area > 12000:
            servo_angle = MID_SERVO + MAX_TURN_DEGREE
        if right_area > 12000:
            servo_angle = MID_SERVO - MAX_TURN_DEGREE

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
    )

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
        turn_dir == "left"
        and target == red_target
        and front_area_black >= 600
        and left_area > WALL_THRESHOLD
    ):
        servo_angle = MID_SERVO - MAX_TURN_DEGREE

    if (
        turn_dir == "right"
        and target == green_target
        and front_area_black >= 600
        and right_area > WALL_THRESHOLD
    ):
        servo_angle = MID_SERVO + MAX_TURN_DEGREE

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

    print(exit_parking_lot_green)
    if exit_parking_lot_green:

        if closest_pillar_area != None:
            print(
                "AAAAAAAAAAAA" + str(closest_pillar_area) + str(closest_pillar_distance)
            )

            servo_angle = MID_SERVO + MAX_TURN_DEGREE  # set the servo to straight
            pw = pwm(servo_angle)

            board.pwm_servo_set_position(0.1, [[2, 1500]])
            board.pwm_servo_set_position(0.1, [[1, pw]])

            board.pwm_servo_set_position(0.1, [[2, DC_SPEED + 20]])
            time.sleep(1)
            servo_angle = MID_SERVO - (MAX_TURN_DEGREE)  # set the servo to straight
            pw = pwm(servo_angle)
            board.pwm_servo_set_position(0.1, [[2, 1500]])
            board.pwm_servo_set_position(0.1, [[1, pw]])

            board.pwm_servo_set_position(0.1, [[2, 1620]])
            time.sleep(1)

            servo_angle = MID_SERVO  # set the servo to straight
            pw = pwm(servo_angle)
            board.pwm_servo_set_position(0.1, [[2, 1500]])
            board.pwm_servo_set_position(0.1, [[1, pw]])

            board.pwm_servo_set_position(0.1, [[2, 1620]])
            time.sleep(1)

            servo_angle = MID_SERVO - MAX_TURN_DEGREE  # set the servo to straight
            pw = pwm(servo_angle)
            board.pwm_servo_set_position(0.1, [[2, 1500]])
            board.pwm_servo_set_position(0.1, [[1, pw]])

            board.pwm_servo_set_position(0.1, [[2, DC_SPEED + 20]])
            time.sleep(1)

            exit_parking_lot_green = False

    if servo_angle > MID_SERVO:
        servo_angle = MID_SERVO + (servo_angle - MID_SERVO) * 0.7

    if not exit_parking_lot_green or exit_parking_lot_red or parking:
        pw = pwm(servo_angle)
        board.pwm_servo_set_position(0.1, [[2, DC_SPEED]])
        board.pwm_servo_set_position(0.1, [[1, pw]])
    if parking:
        servo_angle = MID_SERVO
        pw = pwm(servo_angle)
        board.pwm_servo_set_position(0.1, [[2, 1420]])
        board.pwm_servo_set_position(0.1, [[1, pw]])

        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        buffer = bytearray()

        data = ser.read(256)
        if data:
            buffer += data
            while True:
                idx = find_packet_start(buffer)
                if idx == -1 or len(buffer) - idx < PACKET_LEN:
                    break
                packet = buffer[idx : idx + PACKET_LEN]
                buffer = buffer[idx + PACKET_LEN :]

                parsed = parse_packet(packet)
                if parsed:
                    angles = interpolate_angles(
                        parsed["start_angle"], parsed["end_angle"], 12
                    )
                    # print(
                    # f"\nSpeed: {parsed['speed']:.2f} RPM | Timestamp: {parsed['timestamp']} ms"
                    # )
                    for i, ((dist, conf), angle) in enumerate(
                        zip(parsed["points"], angles)
                    ):
                        print(
                            f"  Pt {i+1:02d}: {angle:.2f}°  {dist} mm  (conf: {conf})"
                        )

                        if angle > 359 and angle < 360 and conf > 160:
                            right_reading = dist
                else:
                    print("Invalid packet")

        print(right_reading)

        if right_reading != None:
            if right_reading < 400:
                servo_angle = MID_SERVO + MAX_TURN_DEGREE
                pw = pwm(servo_angle)
                board.pwm_servo_set_position(0.1, [[2, 1395]])
                board.pwm_servo_set_position(0.1, [[1, pw]])
                time.sleep(1.8)

                servo_angle = MID_SERVO
                pw = pwm(servo_angle)
                board.pwm_servo_set_position(0.1, [[2, 1395]])
                board.pwm_servo_set_position(0.1, [[1, pw]])
                time.sleep(2)

                servo_angle = MID_SERVO
                pw = pwm(servo_angle)
                board.pwm_servo_set_position(0.1, [[2, 1600]])
                board.pwm_servo_set_position(0.1, [[1, pw]])
                time.sleep(1)

                stop()
                break
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
    cv2.imshow("Magenta Mask", img_thresh_magenta)

    cv2.imshow("Camera", im)

    if cv2.waitKey(1) == ord("q") or action_counter >= ACTIONS_TO_STRAIGHT:
        stop()
        break

cv2.destroyAllWindows()
