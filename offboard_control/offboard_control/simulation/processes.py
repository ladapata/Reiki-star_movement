import subprocess
import time

# List of commands to be executed
commands = [
    # Start Micro XRCE-DDS Agent
    "MicroXRCEAgent udp4 -p 8888",

    # Start PX4 SITL simulation
    "cd /root/PX4-Autopilot && PX4_GZ_WORLD=eletroquad make px4_sitl gz_x500_mono_cam",

    # Start QGroundControl
    "cd /root/harpia_ws/ && runuser -l harpia -c 'DISPLAY=:0 /usr/local/bin/QGroundControl.AppImage'",

    # Start ros_gz_bridge
    "cd /root/harpia_ws && ros2 launch offboard_control_bringup ros_gz_bridge.launch.py",

    # Open image view for each camera
    "cd /root/harpia_ws && ros2 run rqt_image_view rqt_image_view --ros-args -r /image:=/camera_forward",
    "cd /root/harpia_ws && ros2 run rqt_image_view rqt_image_view --ros-args -r /image:=/camera_down",
]

# Name of the tmux session
session_name = "simulation"

# Create a new tmux session in the background
subprocess.run(["tmux", "new-session", "-d", "-s", session_name])

# Execute each command in a separate tmux pane
for i, command in enumerate(commands):
    if i == 0:
        # The first command runs in the initial pane
        subprocess.run(["tmux", "send-keys", "-t", f"{session_name}:0", command, "C-m"])
    else:
        # Create a new horizontal pane for subsequent commands
        subprocess.run(["tmux", "split-window", "-h", "-t", session_name])
        subprocess.run(["tmux", "send-keys", "-t", f"{session_name}:.{i}", command, "C-m"])
    
    # Pause to ensure the command is executed before proceeding
    time.sleep(1)

# Add an empty pane in the same window
subprocess.run(["tmux", "split-window", "-v", "-t", session_name])  # Split vertically
subprocess.run(["tmux", "send-keys", "-t", f"{session_name}:.{len(commands)}", "clear", "C-m"])  # Clear the terminal

# Adjust the pane layout to evenly divide the screen
subprocess.run(["tmux", "select-layout", "-t", session_name, "tiled"])




    # <plugin name='gz::sim::systems::Physics' filename='gz-sim-physics-system'/>
    # <plugin name='gz::sim::systems::UserCommands' filename='gz-sim-user-commands-system'/>
    # <plugin name='gz::sim::systems::SceneBroadcaster' filename='gz-sim-scene-broadcaster-system'/>
    # <plugin name='gz::sim::systems::Contact' filename='gz-sim-contact-system'/>