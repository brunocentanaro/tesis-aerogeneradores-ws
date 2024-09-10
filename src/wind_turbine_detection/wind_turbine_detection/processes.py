
#!/usr/bin/env python3

# Import the subprocess and time modules
import subprocess
import time

# List of commands to run
commands = [
    "ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@ignition.msgs.Image /camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo /depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked /depth_camera@sensor_msgs/msg/Image@ignition.msgs.Image"
    #"ros2 run ros_gz_image image_bridge /camera"
]


for command in commands:
    # Each command is run in a new tab of the gnome-terminal
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
    
    # Pause between each command
    time.sleep(1)
