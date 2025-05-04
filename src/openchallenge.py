# libraries
import cv2  # for image processing
import numpy as np  # for image manipulation
import time
import ros_robot_controller_sdk as rrc


from picamera2 import Picamera2
import RPi.GPIO as GPIO

board = rrc.Board()
# movement constants
MID_SERVO = 63
MAX_TURN_DEGREE = 20
DC_SPEED = 1376


# proportion constants for the servo motor angle (PID steering)
PD = 0.0015
PG = 0.0025
# no integral value


# ROI constants
ROI_LEFT_BOT = [0, 220, 100, 300]
ROI_RIGHT_BOT = [540, 220, 640, 300]

ROI4 = [270, 330, 370, 370]


# color threshold constants (in HSV)
# LOWER_BLUE = np.array([104, 65, 70])
# UPPER_BLUE = np.array([132, 205, 185])
# LOWER_ORANGE1 = np.array([155, 59, 70])
# UPPER_ORANGE1 = np.array([180, 255, 255])
# LOWER_ORANGE2 = np.array([0, 59, 70])
# UPPER_ORANGE2 = np.array([8, 255, 255])

LOWER_ORANGE1 = np.array([180, 100, 100])
UPPER_ORANGE1 = np.array([180, 255, 255])
LOWER_ORANGE2 = np.array([0, 80, 80])
UPPER_ORANGE2 = np.array([20, 255, 255])

LOWER_BLUE = np.array([101, 20, 30])
UPPER_BLUE = np.array([125, 255, 255])

LOWER_BLACK_THRESHOLD = np.array([0, 0, 0])
UPPER_BLACK_THRESHOLD = np.array([180, 255, 80])


# camera settings
WIDTH = 640
HEIGHT = 480
POINTS = [(115, 200), (525, 200), (640, 370), (0, 370)]


# limiting constants
MAX_TURNS = 4
ACTIONS_TO_STRAIGHT = 80
WALL_THRESHOLD = 50
NO_WALL_THRESHOLD = 25
TURN_ITER_LIMIT = 30
LINE_THRESHOLD = 30


# incrementing variables
action_counter = 0
turn_counter = 0
turn_length_counter = 0
last_turn_count = 0

# dynamic variables
track_dir = None
done_turning = False
turn_dir = None
last_difference = 0
current_difference = 0
seen_line = False

# initialize servo angle variable
servo_angle = 0


# initialization of camera global variables
im = None
image = None


# camera setup
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 30
picam2.configure("preview")
picam2.start()


# utility function for pwm angle calculation
def pwm(degree):  # angle must be adjusted to pwm angle for servo
    """
    Convert angle in degrees to pulse width modulation (PWM) for servo to understand

    Parameters:
    degree (int): The angle the servo should turn to.

    Returns:
    int: The equivalent pwm for the desired angle.
    """

    return round(degree * 11.1 + 500)


# utility function for drawing ROI
def drawROI(ROI):  # draw the rectangular ROI on the image
    """
    Displays/draws a rectanngular region of interest (ROI) on the OpenCV window

    Parameters:
    ROI (list): A list of 2 coodinates detailing the top left and the bottom right of the ROI

    Returns:

    """

    image = cv2.line(im, (ROI[0], ROI[1]), (ROI[2], ROI[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI[0], ROI[1]), (ROI[0], ROI[3]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI[2], ROI[3]), (ROI[2], ROI[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI[2], ROI[3]), (ROI[0], ROI[3]), (0, 255, 255), 4)


# utility function for stopping the robot
def stop():
    """
    Sets servo and DC motor straight and closing the OpenCV window

    Parameters:

    Returns:

    """

    time.sleep(0.02)
    board.pwm_servo_set_position(0.1, [[2, 1500]])

    board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])

    cv2.destroyAllWindows()


# setting servo angle to straight and the motor to stationary
board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
# turn servo to mid
board.pwm_servo_set_position(0.1, [[2, 1500]])  # arm the esc motor
time.sleep(0.5)


# main loop
while True:

    # exit if button is pressed

    # setup camera frame
    im = picam2.capture_array()
    input = np.float32(POINTS)
    output = np.float32(
        [(0, 0), (WIDTH - 1, 0), (WIDTH - 1, HEIGHT - 1), (0, HEIGHT - 1)]
    )

    # convert to hsv
    img_hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

    # find black thresholds/mask and store them in a variable
    img_thresh = cv2.inRange(img_hsv, LOWER_BLACK_THRESHOLD, UPPER_BLACK_THRESHOLD)

    # find black contours in left and right regions

    left_contours_bot, hierarchy = cv2.findContours(
        img_thresh[
            ROI_LEFT_BOT[1] : ROI_LEFT_BOT[3], ROI_LEFT_BOT[0] : ROI_LEFT_BOT[2]
        ],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )
    right_contours_bot, hierarchy = cv2.findContours(
        img_thresh[
            ROI_RIGHT_BOT[1] : ROI_RIGHT_BOT[3], ROI_RIGHT_BOT[0] : ROI_RIGHT_BOT[2]
        ],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )

    # define variables for left and right ROI contour areas
    left_area_bot = 0

    right_area_bot = 0

    # loop to find largest contours

    for i in range(len(left_contours_bot)):
        cnt = left_contours_bot[i]
        area = cv2.contourArea(cnt)
        left_area_bot = max(area, left_area_bot)

    for i in range(len(right_contours_bot)):
        cnt = right_contours_bot[i]
        area = cv2.contourArea(cnt)
        right_area_bot = max(area, right_area_bot)

    # combine areas on each side
    right_area = right_area_bot
    left_area = left_area_bot

    # find blue thresholds/mask and store them in a variable
    b_mask = cv2.inRange(img_hsv, LOWER_BLUE, UPPER_BLUE)

    # find blue contours to detect the lines on the mat
    contours_blue = cv2.findContours(
        b_mask[ROI4[1] : ROI4[3], ROI4[0] : ROI4[2]],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )[-2]

    # find orange thresholds/mask and store them in a variable
    o_mask = cv2.bitwise_or(
        cv2.inRange(img_hsv, LOWER_ORANGE1, UPPER_ORANGE1),
        cv2.inRange(img_hsv, LOWER_ORANGE2, UPPER_ORANGE2),
    )

    # find orange contours to detect the lines on the mat
    contours_orange = cv2.findContours(
        o_mask[ROI4[1] : ROI4[3], ROI4[0] : ROI4[2]],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )[-2]

    # iterate through the blue and orange contours and store the maximum area
    max_blue_area = 0
    max_orange_area = 0

    for i in range(len(contours_orange)):
        cnt = contours_orange[i]
        max_orange_area = max(cv2.contourArea(cnt), max_orange_area)
        cnt[:, :, 0] += ROI4[0]  # Add X offset
        cnt[:, :, 1] += ROI4[1]  # Add Y offset
        cv2.drawContours(
            im, contours_orange, i, (255, 255, 0), 1
        )  # draw the contours for debug utility
    for i in range(len(contours_blue)):
        cnt = contours_blue[i]
        max_blue_area = max(cv2.contourArea(cnt), max_blue_area)
        cnt[:, :, 0] += ROI4[0]  # Add X offset
        cnt[:, :, 1] += ROI4[1]  # Add Y offset
        cv2.drawContours(
            im, contours_blue, i, (255, 255, 0), 1
        )  # draw the contours for debug utility

    if max_blue_area >= LINE_THRESHOLD and track_dir != "right" and seen_line == False:
        track_dir = "left"
        turn_counter += 1
        seen_line = True
    if max_orange_area >= LINE_THRESHOLD and track_dir != "left" and seen_line == False:
        track_dir = "right"
        turn_counter += 1
        seen_line = True

    if turn_dir == "left" and not max_blue_area >= LINE_THRESHOLD:
        if left_area > WALL_THRESHOLD:
            turn_dir = None
            seen_line = False

            last_turn_count = turn_counter

    if turn_dir == "right" and not max_orange_area >= LINE_THRESHOLD:
        if right_area > WALL_THRESHOLD:
            turn_dir = None
            seen_line = False
    if left_area < NO_WALL_THRESHOLD and turn_dir == None and not track_dir == "right":
        track_dir = "left"

        tufn_dir = "left"

    elif (
        right_area < NO_WALL_THRESHOLD and turn_dir == None and not track_dir == "left"
    ):
        track_dir = "right"

        tufn_dir = "right"

    elif turn_dir == None:

        # if in the straight section, calculate the current_difference between the contours in the left and right area

        current_difference = left_area - right_area

        # calculate steering amount using preportional-derivative steering
        # multiply the current_difference by a constant variable and add the projected error multiplied by another constant
        # this method gives stable and smooth steering

        servo_angle = MID_SERVO + (
            current_difference * PG + (current_difference - last_difference) * PD
        )

    if (
        turn_counter == MAX_TURNS and not done_turning
    ):  # if the total turns has surpassed the amount required, increment the action counter by 1
        action_counter += 1  # this is to ensure that the robot stops in the middle of the straight section

    # set the last current_difference equal to the current current_difference for derivative steering
    last_difference = current_difference
    print(turn_counter)

    # if the steering variable is higher than the max turn degree for the servo, set it to the max turn degree
    if (servo_angle) > MID_SERVO + MAX_TURN_DEGREE:
        servo_angle = MID_SERVO + MAX_TURN_DEGREE
    if (servo_angle) < MID_SERVO - MAX_TURN_DEGREE:
        servo_angle = MID_SERVO - MAX_TURN_DEGREE
    if turn_dir == "right":  # calculate the servo angle for the current turn
        servo_angle = MID_SERVO + (MAX_TURN_DEGREE / 1.8)
        print(turn_dir)
    elif turn_dir == "left":  # calculate the servo angle for the current turn
        servo_angle = MID_SERVO - (MAX_TURN_DEGREE / 1.8)
        print(turn_dir)
    # move the motors using the variables
    pw = pwm(servo_angle)
    board.pwm_servo_set_position(0.1, [[2, DC_SPEED]])

    board.pwm_servo_set_position(0.1, [[1, pw]])

    # draw the ROIs
    drawROI(ROI_LEFT_BOT)
    drawROI(ROI_RIGHT_BOT)
    drawROI(ROI4)

    # display the camera

    cv2.imshow("Camera", im)

    # if the number of actions to the straight section has been met, stop the car
    if cv2.waitKey(1) == ord("q") or action_counter >= ACTIONS_TO_STRAIGHT:
        stop()
        break

cv2.destroyAllWindows()
