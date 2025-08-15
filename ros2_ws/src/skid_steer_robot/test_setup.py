#!/usr/bin/env python3

import subprocess
import sys
import os

def check_package(package_name):
    """Check if a ROS2 package is available."""
    try:
        result = subprocess.run(['ros2', 'pkg', 'list'], 
                              capture_output=True, text=True, check=True)
        return package_name in result.stdout
    except subprocess.CalledProcessError:
        return False

def check_gazebo_garden():
    """Check if Gazebo Garden (version 6) is available."""
    try:
        # Check if ign command exists
        result = subprocess.run(['which', 'ign'], 
                              capture_output=True, text=True, check=True)
        if result.stdout.strip():
            # Try to run ign gazebo --version to verify it works
            version_result = subprocess.run(['ign', 'gazebo', '--version'], 
                                          capture_output=True, text=True, check=True)
            return '6.' in version_result.stdout or 'Garden' in version_result.stdout
        return False
    except (subprocess.CalledProcessError, FileNotFoundError):
        return False

def check_gazebo_fortress():
    """Check if Gazebo Fortress (version 7) is available."""
    try:
        # Check if ign command exists
        result = subprocess.run(['which', 'ign'], 
                              capture_output=True, text=True, check=True)
        if result.stdout.strip():
            # Try to run ign gazebo --version to verify it works
            version_result = subprocess.run(['ign', 'gazebo', '--version'], 
                                          capture_output=True, text=True, check=True)
            return '7.' in version_result.stdout or 'Fortress' in version_result.stdout
        return False
    except (subprocess.CalledProcessError, FileNotFoundError):
        return False

def check_gazebo_classic():
    """Check if Gazebo classic is available."""
    try:
        # Check if gz command exists
        result = subprocess.run(['which', 'gz'], 
                              capture_output=True, text=True, check=True)
        if result.stdout.strip():
            # Try to run gz help to verify it works
            help_result = subprocess.run(['gz', 'help'], 
                                       capture_output=True, text=True, check=True)
            return True
        return False
    except (subprocess.CalledProcessError, FileNotFoundError):
        return False

def main():
    print("Checking ROS2 Humble and Gazebo setup...")
    print("=" * 50)
    
    # Check ROS2
    print("ROS2 Humble:")
    if check_package('ros_gz'):
        print("  ✓ ros_gz package available")
    else:
        print("  ✗ ros_gz package not found")
        print("    Install with: sudo apt install ros-humble-ros-gz")
    
    if check_package('controller_manager'):
        print("  ✓ controller_manager package available")
    else:
        print("  ✗ controller_manager package not found")
        print("    Install with: sudo apt install ros-humble-controller-manager")
    
    if check_package('diff_drive_controller'):
        print("  ✓ diff_drive_controller package available")
    else:
        print("  ✗ diff_drive_controller package not found")
        print("    Install with: sudo apt install ros-humble-diff-drive-controller")
    
    if check_package('teleop_twist_keyboard'):
        print("  ✓ teleop_twist_keyboard package available")
    else:
        print("  ✗ teleop_twist_keyboard package not found")
        print("    Install with: sudo apt install ros-humble-teleop-twist-keyboard")
    
    # Check Gazebo
    print("\nGazebo:")
    if check_gazebo_garden():
        print("  ✓ Gazebo Garden (version 6) available")
        gazebo_type = "garden"
    elif check_gazebo_fortress():
        print("  ✓ Gazebo Fortress (version 7) available")
        gazebo_type = "fortress"
    elif check_gazebo_classic():
        print("  ✓ Gazebo Classic available")
        gazebo_type = "classic"
    else:
        print("  ✗ No Gazebo found")
        print("    Install Gazebo Classic with: sudo apt install gazebo")
        print("    Or install Gazebo Garden/Fortress with: sudo apt install gazebo-garden")
        gazebo_type = "none"
    
    print("\n" + "=" * 50)
    print("Setup check complete!")
    
    if gazebo_type == "fortress":
        print("\nGazebo Fortress detected! Perfect for ROS2 Humble.")
        print("\nTo build and test the package:")
        print("1. cd ~/ros2_ws")
        print("2. colcon build --packages-select skid_steer_robot")
        print("3. source install/setup.bash")
        print("4. ros2 launch skid_steer_robot test_gazebo.launch.py")
        print("5. ros2 launch skid_steer_robot simple_sim.launch.py")
    elif gazebo_type == "garden":
        print("\nGazebo Garden detected! Great for ROS2 Humble.")
        print("\nTo build and test the package:")
        print("1. cd ~/ros2_ws")
        print("2. colcon build --packages-select skid_steer_robot")
        print("3. source install/setup.bash")
        print("4. ros2 launch skid_steer_robot test_gazebo.launch.py")
        print("5. ros2 launch skid_steer_robot simple_sim.launch.py")
    elif gazebo_type == "classic":
        print("\nGazebo Classic detected! The package will work but may need adjustments.")
        print("\nTo build and test the package:")
        print("1. cd ~/ros2_ws")
        print("2. colcon build --packages-select skid_steer_robot")
        print("3. source install/setup.bash")
        print("4. ros2 launch skid_steer_robot test_gazebo.launch.py")
        print("5. ros2 launch skid_steer_robot simple_sim.launch.py")
    else:
        print("\nNo Gazebo found. Please install either Gazebo Classic or Gazebo Garden/Fortress first.")

if __name__ == '__main__':
    main()
