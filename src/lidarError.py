import struct
import time
import serial
import math
import threading
import ros_robot_controller_sdk as rrc

board = rrc.Board()

# --- Constants and Helper Functions ---
PORT = "/dev/ttyAMA0"
BAUD = 230400
PACKET_LEN = 47
MID_SERVO = 64
MAX_TURN_DEGREE = 26
DC_SPEED = 1390
GYROPG = 0.5


def pwm(degree):
    return round(degree * 11.1 + 500)


def find_packet_start(buffer):
    for i in range(len(buffer) - 1):
        if buffer[i] == 0x54 and (buffer[i + 1] & 0xFF) == 0x2C:
            return i
    return -1


def detect_angle_reset(current_angle, previous_angle):
    if previous_angle is None:
        return False
    return previous_angle > 300 and current_angle < 60


def parse_packet(packet):
    if len(packet) != PACKET_LEN:
        return None
    start_angle = struct.unpack_from("<H", packet, 4)[0] / 100.0
    measurements = []
    for i in range(12):
        offset = 6 + i * 3
        dist = struct.unpack_from("<H", packet, offset)[0]
        confidence = packet[offset + 2]
        measurements.append((dist, confidence))
    end_angle = struct.unpack_from("<H", packet, 42)[0] / 100.0
    return {"start_angle": start_angle, "end_angle": end_angle, "points": measurements}


def interpolate_angles(start, end, count):
    angle_range = (end - start + 360) % 360
    step = angle_range / (count - 1)
    return [(start + i * step) % 360 for i in range(count)]


def filter_wall_points(points, x_min=5, x_max=100, y_min=-25, y_max=0):
    wall_points = []
    for x, y in points:
        if x_min <= x <= x_max and y_min <= y <= y_max:
            wall_points.append((x, y))
    return wall_points


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


def main():
    """
    Diagnostic version: No plotting, console output only.
    Checks if data processing is fast without plotting overhead.
    """
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    buffer = bytearray()

    print("Starting diagnostic test (console only)...")

    current_scan_points = []
    previous_angle = None
    rotation_count = 0

    ser.reset_input_buffer()
    servo_angle = MID_SERVO
    while True:
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
                                    y_min=-25,
                                    y_max=0,
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
                                        print(servo_angle)
                                        servo_angle = MID_SERVO - (angle_error * GYROPG)

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
                            current_scan_points.append((x, y))

                        previous_angle = angle

                buffer = buffer[idx + PACKET_LEN :]

            if (servo_angle) > MID_SERVO + MAX_TURN_DEGREE:
                servo_angle = MID_SERVO + MAX_TURN_DEGREE
            elif (servo_angle) < MID_SERVO - MAX_TURN_DEGREE:
                servo_angle = MID_SERVO - MAX_TURN_DEGREE

            board.pwm_servo_set_position(0.1, [[2, DC_SPEED]])
            board.pwm_servo_set_position(0.1, [[1, pwm(servo_angle)]])

        except KeyboardInterrupt:
            print("\nProgram ended.")
            board.pwm_servo_set_position(0.1, [[2, 1500]])
            board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
            ser.close()
            break
        except Exception as e:
            print(f"An error occurred: {e}")
            board.pwm_servo_set_position(0.1, [[2, 1500]])
            board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
            ser.close()
            break


if __name__ == "__main__":
    main()
