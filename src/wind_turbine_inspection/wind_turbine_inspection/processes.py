
#!/usr/bin/env python3

# Import the subprocess and time modules
import subprocess
import time

# List of commands to run
commands = [
    "MicroXRCEAgent udp4 -p 8888",

    # "cd ~/PX4-Autopilot && make px4_sitl gz_x500_gimbal",
    "cd ~/QGroundControl && ./QGroundControl.AppImage",
]

# Loop through each command in the list
for command in commands:
    # Each command is run in a new tab of the gnome-terminal
    subprocess.run(["gnome-terminal", "--tab", "--",
                   "bash", "-c", command + "; exec bash"])

    # Pause between each command
    time.sleep(1)
