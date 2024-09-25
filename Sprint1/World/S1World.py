import os
import subprocess

# Define the paths to your Gazebo models and world files
repo_path = "~/ros2_ws/src/Sprint1/World/gazebo_models_worlds_collection/worlds/office_env_large.world"  # Update this with the actual path where you cloned the repo
world_file = os.path.join(repo_path, "worlds", "office_env_large.world")  # Assuming the world file is named office_environment.world
turtlebot3_model = "model://turtlebot3_burger"  # Change to 'waffle' or 'waffle_pi' if using a different TurtleBot3 model

# Set the environment variable for Gazebo to find the models
os.environ["GAZEBO_MODEL_PATH"] = os.path.join(repo_path, "models")

# Command to run Gazebo with the specified world file
gazebo_command = ["gazebo", world_file, "--verbose"]

# Command to spawn the TurtleBot3 into the world
spawn_command = [
    "ros2", "run", "gazebo_ros", "spawn_entity.py",
    "-entity", "turtlebot3",
    "-database", turtlebot3_model,
    "-x", "0", "-y", "0", "-z", "0.1"  # Adjust the spawn position if needed
]

# Start the Gazebo world
print("Starting Gazebo with the office environment...")
gazebo_process = subprocess.Popen(gazebo_command)

# Wait a bit to ensure Gazebo is up and running
print("Waiting for Gazebo to initialize...")
subprocess.run(["sleep", "5"])  # Sleep for 5 seconds

# Spawn the TurtleBot3 in the world
print("Spawning TurtleBot3 in the world...")
subprocess.run(spawn_command)

# Wait for Gazebo to close
gazebo_process.wait()
