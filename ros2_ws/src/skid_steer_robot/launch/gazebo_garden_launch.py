#!/usr/bin/env python3

import os
import subprocess
import time
import signal
import sys
from ament_index_python.packages import get_package_share_directory

def main():
    pkg_share = get_package_share_directory('skid_steer_robot')
    world_file = os.path.join(pkg_share, 'worlds', 'world.sdf')
    
    print(f"Starting Gazebo Garden with world: {world_file}")
    
    # Start Gazebo
    try:
        gazebo_process = subprocess.Popen(
            ['ign', 'gazebo', '-r', world_file],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        
        print("Gazebo Garden started. Waiting for it to initialize...")
        time.sleep(5)  # Wait for Gazebo to start
        
        # Spawn the robot using ign service call
        print("Spawning skid steer robot...")
        spawn_cmd = [
            'ign', 'service', '-s', '/world/skid_steer_world/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000',
            '--req', 'sdf_filename: "' + os.path.join(pkg_share, 'model.sdf') + '", name: "skid_steer_robot"'
        ]
        
        spawn_result = subprocess.run(spawn_cmd, capture_output=True, text=True)
        if spawn_result.returncode == 0:
            print("Robot spawned successfully!")
        else:
            print(f"Failed to spawn robot: {spawn_result.stderr}")
        
        print("\nGazebo Garden is running with the skid steer robot!")
        print("Press Ctrl+C to exit...")
        
        # Wait for user to interrupt
        try:
            gazebo_process.wait()
        except KeyboardInterrupt:
            print("\nShutting down Gazebo...")
            gazebo_process.terminate()
            gazebo_process.wait()
            print("Gazebo stopped.")
            
    except FileNotFoundError:
        print("Error: 'ign' command not found. Make sure Gazebo Garden is installed.")
        sys.exit(1)
    except Exception as e:
        print(f"Error starting Gazebo: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
