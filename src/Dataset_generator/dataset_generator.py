

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import random
import pandas as pd
import numpy as np
from datetime import datetime
import json

class DroneManeuverAnalyzer:
    def __init__(self, connection_string='tcp:127.0.0.1:57600'):
        self.connection_string = connection_string
        self.vehicle = None
        self.maneuver_data = []
        
    def connect_drone(self):
        print('Connecting to vehicle...')
        try:
            self.vehicle = connect(self.connection_string, wait_ready=True)
            print("Connected successfully!")
            return True
        except Exception as e:
            print(f"Connection failed: {str(e)}")
            return False

    def arm_and_takeoff(self, target_altitude):
        print("Basic pre-arm checks...")
        while not self.vehicle.is_armable:
            time.sleep(1)

        print("Arming motors")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            time.sleep(1)

        print(f"Taking off to {target_altitude}m!")
        self.vehicle.simple_takeoff(target_altitude)

        while True:
            altitude = self.vehicle.location.global_relative_frame.alt
            if altitude >= target_altitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

    def set_velocity(self, velocity_x, velocity_y, velocity_z):
        """Send velocity command to vehicle"""
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,
            0, 0, 0,
            velocity_x, velocity_y, velocity_z,
            0, 0, 0,
            0, 0)
        self.vehicle.send_mavlink(msg)

    def test_maneuver(self, initial_velocity, maneuver_type='turn'):
        """Test a specific maneuver at given velocity"""
        try:
            # Accelerate to initial velocity
            print(f"\nAccelerating to {initial_velocity:.1f} m/s")
            self.set_velocity(initial_velocity, 0, 0)
            
            # Wait to reach velocity
            time.sleep(5)
            
            # Record initial state
            initial_position = self.vehicle.location.global_relative_frame
            initial_heading = self.vehicle.heading
            
            # Execute maneuver
            if maneuver_type == 'turn':
                print("Executing turn...")
                self.vehicle.message_factory.command_long_send(
                    0, 0,
                    mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                    0, 90, 45, 1, 0, 0, 0, 0)  # 90-degree turn
            else:  # stop
                print("Executing stop...")
                self.set_velocity(0, 0, 0)
            
            # Monitor response
            start_time = time.time()
            max_response_time = 5  # seconds to complete maneuver
            completion_time = None
            response_distance = 0
            success = False
            
            while time.time() - start_time < max_response_time:
                current_position = self.vehicle.location.global_relative_frame
                current_velocity = self.vehicle.airspeed
                
                # Calculate distance traveled
                dlat = current_position.lat - initial_position.lat
                dlon = current_position.lon - initial_position.lon
                response_distance = math.sqrt((dlat*111319)**2 + (dlon*111319*math.cos(initial_position.lat*math.pi/180))**2)
                
                if maneuver_type == 'turn':
                    # Check if turn completed
                    heading_change = abs(self.vehicle.heading - initial_heading)
                    if heading_change > 80:  # Consider turn complete at 80 degrees
                        completion_time = time.time() - start_time
                        success = response_distance < initial_velocity * 2  # Success if overshoot less than 2s worth of travel
                        break
                else:  # stop
                    # Check if stop completed
                    if current_velocity < 0.5:  # Consider stopped below 0.5 m/s
                        completion_time = time.time() - start_time
                        success = response_distance < initial_velocity * 3  # Success if stopping distance less than 3s worth of travel
                        break
                
                time.sleep(0.1)
            
            # Record maneuver data
            maneuver_data = {
                'timestamp': datetime.now().isoformat(),
                'maneuver_type': maneuver_type,
                'initial_velocity': initial_velocity,
                'response_distance': response_distance,
                'completion_time': completion_time if completion_time else max_response_time,
                'success': success,
                'final_velocity': self.vehicle.airspeed
            }
            
            self.maneuver_data.append(maneuver_data)
            
            print(f"Maneuver completed:")
            print(f"Success: {success}")
            print(f"Response distance: {response_distance:.1f}m")
            print(f"Completion time: {completion_time if completion_time else 'Not completed':.1f}s")
            
            return maneuver_data
            
        except Exception as e:
            print(f"Error during maneuver: {str(e)}")
            return None

    def run_test_sequence(self, num_tests=20):
        """Run a sequence of maneuver tests"""
        try:
            # Take off
            self.arm_and_takeoff(30)
            
            # Test different velocities for both turns and stops
            velocities = np.linspace(5, 30, num_tests//2)  # Test velocities from 5 to 30 m/s
            maneuvers = ['turn', 'stop']
            
            for velocity in velocities:
                for maneuver in maneuvers:
                    print(f"\nTesting {maneuver} at {velocity:.1f} m/s")
                    
                    # Return to center position
                    self.vehicle.mode = VehicleMode("GUIDED")
                    center_location = LocationGlobalRelative(
                        self.vehicle.home_location.lat,
                        self.vehicle.home_location.lon,
                        30
                    )
                    self.vehicle.simple_goto(center_location)
                    time.sleep(10)
                    
                    # Execute test
                    self.test_maneuver(velocity, maneuver)
                    
                    # Short pause between tests
                    time.sleep(5)
            
            # Return to launch
            print("\nTests completed, returning to launch")
            self.vehicle.mode = VehicleMode("RTL")
            time.sleep(20)
            
            # Save and analyze data
            self.save_data()
            self.analyze_results()
            
        except Exception as e:
            print(f"Error during test sequence: {str(e)}")
        finally:
            self.vehicle.close()

    def save_data(self):
        """Save collected data"""
        if not self.maneuver_data:
            print("No data to save!")
            return
        
        # Save detailed CSV
        df = pd.DataFrame(self.maneuver_data)
        df.to_csv('maneuver_analysis.csv', index=False)
        print("Data saved to maneuver_analysis.csv")
        
        # Save processed JSON for model training
        with open('maneuver_analysis.json', 'w') as f:
            json.dump(self.maneuver_data, f, indent=2)
        print("Data saved to maneuver_analysis.json")

    def analyze_results(self):
        """Analyze the collected data"""
        df = pd.DataFrame(self.maneuver_data)
        
        print("\nManeuver Analysis Results:")
        print("=========================")
        
        # Analyze by maneuver type
        for maneuver in ['turn', 'stop']:
            maneuver_df = df[df['maneuver_type'] == maneuver]
            
            print(f"\n{maneuver.upper()} Analysis:")
            print(f"Total tests: {len(maneuver_df)}")
            print(f"Success rate: {(maneuver_df['success'].mean() * 100):.1f}%")
            
            # Find velocity threshold for reliable execution
            success_df = maneuver_df[maneuver_df['success'] == True]
            if not success_df.empty:
                max_success_velocity = success_df['initial_velocity'].max()
                print(f"Maximum successful velocity: {max_success_velocity:.1f} m/s")
            
            # Calculate correlation between velocity and response distance
            correlation = maneuver_df['initial_velocity'].corr(maneuver_df['response_distance'])
            print(f"Velocity-Distance Correlation: {correlation:.3f}")

def main():
    print("""
    DRONE MANEUVER CAPABILITY ANALYZER
    
    This script will:
    1. Test drone's ability to turn and stop at different velocities
    2. Collect performance data for each maneuver
    3. Analyze success rates and limitations
    4. Generate prediction dataset
    
    Make sure Mission Planner SITL is running!
    """)
    
    num_tests = int(input("Enter number of tests per maneuver type (default=10): ") or "10")
    
    analyzer = DroneManeuverAnalyzer()
    if analyzer.connect_drone():
        analyzer.run_test_sequence(num_tests * 2)  # *2 for both turn and stop tests

if __name__ == "__main__":
    main()