#!/usr/bin/env python3
import subprocess, threading, math

CONTAINER = "MentorPi"
SHELL     = "/bin/zsh"
USER      = "ubuntu"
WORKDIR   = "/home/ubuntu"
ENV_SRC   = "source /home/ubuntu/.zshrc"
TOPIC     = "/imu/rpy/filtered"  # geometry_msgs/Vector3Stamped

def check_topic_exists():
    cmd = f"{ENV_SRC} && ros2 topic list"
    r = subprocess.run(
        ["docker","exec","-u",USER,"-w",WORKDIR,CONTAINER,SHELL,"-c",cmd],
        capture_output=True, text=True
    )
    return TOPIC in r.stdout

def listen_to_imu_z():
    # Stream the topic and parse only the 'vector.z' line
    cmd = f"{ENV_SRC} && ros2 topic echo {TOPIC}"
    p = subprocess.Popen(
        ["docker","exec","-u",USER,"-w",WORKDIR,CONTAINER,SHELL,"-c",cmd],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )

    in_vector_block = False
    for line in p.stdout:
        s = line.strip()
        if not s:
            continue
        if s.startswith("vector:"):
            in_vector_block = True
            continue
        if in_vector_block:
            if s.startswith("z:"):
                try:
                    z_rad = float(s.split(":",1)[1].strip())
                    z_deg = math.degrees(z_rad)
                    print(f"yaw z = {z_rad:.6f} rad  ({z_deg:.2f}°)")
                except ValueError:
                    pass
                in_vector_block = False
            elif s[0].isalpha() and not s.startswith(("x:","y:")):
                # new section started; leave vector block
                in_vector_block = False

if __name__ == "__main__":
    if check_topic_exists():
        print("IMU topic detected; listening for yaw (z)…")
        t = threading.Thread(target=listen_to_imu_z, daemon=True)
        t.start()
        while t.is_alive():
            t.join(1)
    else:
        print(f"Topic {TOPIC} not found.")
