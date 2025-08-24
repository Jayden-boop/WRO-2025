import serial
import struct
import time
import matplotlib

matplotlib.use("TkAgg")  # Use 'Agg' if running headless
import matplotlib.pyplot as plt
import math

PORT = "/dev/ttyAMA0"
BAUD = 230400
PACKET_LEN = 47


def find_packet_start(buffer):
    for i in range(len(buffer) - 1):
        if buffer[i] == 0x54 and (buffer[i + 1] & 0xFF) == 0x2C:
            return i
    return -1


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


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    buffer = bytearray()

    plt.ion()
    fig = plt.figure(figsize=(8, 8))

    print("Plotting objects within 100 cm...")

    all_points = []
    plot_start_time = time.time()
    last_plot_time = time.time()

    while True:
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
                    for (dist, conf), angle in zip(parsed["points"], angles):
                        if 0 < dist < 1000:  # within 100 cm (1000 mm)
                            angle_rad = math.radians(angle)
                            x = dist * math.cos(angle_rad) / 10.0  # mm to cm
                            y = dist * math.sin(angle_rad) / 10.0
                            all_points.append((x, y))

        # Plot every second
        current_time = time.time()
        if current_time - last_plot_time >= 1.0:
            plt.clf()
            if all_points:
                xs, ys = zip(*all_points)
                plt.scatter(xs, ys, s=5, label="Detected Points")
            # Add 100 cm boundary circle
            circle = plt.Circle(
                (0, 0),
                100,
                color="gray",
                fill=False,
                linestyle="--",
                label="100 cm Range",
            )
            plt.gca().add_patch(circle)
            plt.title("Objects within 100 cm (Live, 30s window)")
            plt.xlabel("X (cm)")
            plt.ylabel("Y (cm)")
            plt.axis("equal")
            plt.grid(True)
            plt.legend()
            plt.pause(0.001)
            last_plot_time = current_time

        # Reset data every 30 seconds
        if current_time - plot_start_time >= 1.0:
            all_points = []
            plot_start_time = current_time
            print("Resetting accumulated data after 30s.")

        time.sleep(0.01)


if __name__ == "__main__":
    main()
