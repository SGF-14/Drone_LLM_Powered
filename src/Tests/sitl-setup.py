from collections.abc import MutableMapping
import collections
collections.MutableMapping = MutableMapping

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import time
import math
import random
import pandas as pd
import numpy as np
from datetime import datetime

class DroneTurnAnalyzer:
    def __init__(self, connection_string='tcp:127.0.0.1:5763'):
        self.connection_string = connection_string
        self.vehicle = None
        self.data = []
        self.turn_data = []
        
    def connect_drone(self):
        """Connect to the drone in SITL"""
        print('Connecting to vehicle...')
        try:
            self.vehicle = connect(self.connection_string, wait_ready=True)
            print("Connected successfully!")
            return True
        except Exception as e:
            print(f"Connection failed: {str(e)}")
            return False

    def arm_and_takeoff(self, target_altitude):
        """Arm and takeoff to specified altitude"""
        print("Basic pre-arm checks...")
        while not self.vehicle.is_armable:
            print("Waiting for vehicle to initialize...")
            time.sleep(1)

        print("Arming motors")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print("Waiting for arming...")
            time.sleep(1)

        print(f"Taking off to {target_altitude}m!")
        self.vehicle.simple_takeoff(target_altitude)

        while True:
            altitude = self.vehicle.location.global_relative_frame.alt
            print(f"Altitude: {altitude:.1f}m")
            if altitude >= target_altitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

    def get_distance_metres(self, location1, location2):
        """Get distance between two LocationGlobal objects in meters"""
        dlat = location2.lat - location1.lat
        dlong = location2.lon - location1.lon
        
        return math.sqrt((dlat*111319)**2 + (dlong*111319*math.cos(location1.lat*math.pi/180))**2)

    def send_velocity_command(self, velocity_x, velocity_y, velocity_z):
        """Send velocity command to vehicle"""
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not used)
            0, 0)    # yaw, yaw_rate (not used)
        
        self.vehicle.send_mavlink(msg)

    def collect_turn_data(self, initial_location, current_location, target_location, velocity, heading_change):
        """Collect data about the turn performance"""
        actual_distance = self.get_distance_metres(initial_location, current_location)
        target_distance = self.get_distance_metres(initial_location, target_location)
        overshoot = actual_distance - target_distance
        
        turn_data = {
            'timestamp': datetime.now().isoformat(),
            'velocity': velocity,
            'heading_change': heading_change,
            'target_distance': target_distance,
            'actual_distance': actual_distance,
            'overshoot': overshoot,
            'turn_success': overshoot < 10,  # Consider turn successful if overshoot < 10m
            'latitude': current_location.lat,
            'longitude': current_location.lon,
            'altitude': current_location.alt
        }
        
        self.turn_data.append(turn_data)
        return turn_data

    def execute_random_velocity_turn(self):
        """Execute a single random velocity turn test"""
        # Generate random velocity between 5 and 25 m/s
        velocity = random.uniform(5, 25)
        print(f"\nTesting with velocity: {velocity:.1f} m/s")
        
        # Set initial velocity
        self.send_velocity_command(velocity, 0, 0)
        time.sleep(5)  # Allow drone to reach velocity
        
        # Record initial position
        initial_location = self.vehicle.location.global_relative_frame
        initial_heading = self.vehicle.heading
        
        # Calculate target turn position (90 degrees turn)
        heading_change = 90
        target_distance = velocity * 2  # Expected distance to complete turn
        
        # Calculate target position
        target_lat = initial_location.lat + (math.sin(math.radians(initial_heading)) * target_distance / 111319)
        target_lon = initial_location.lon + (math.cos(math.radians(initial_heading)) * target_distance / 111319)
        target_location = LocationGlobalRelative(target_lat, target_lon, initial_location.alt)
        
        print("Executing sudden turn...")
        # Execute turn command
        self.vehicle.message_factory.command_long_send(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            heading_change,  # param 1 - target heading
            25,     # param 2 - turn rate (deg/s)
            1,      # param 3 - direction (1=clockwise)
            0,      # relative offset
            0, 0, 0 # params 5-7 (not used) 
        )
        
        # Wait for turn to complete and collect data
        turn_start_time = time.time()
        while time.time() - turn_start_time < 5:  # Monitor turn for 5 seconds
            current_location = self.vehicle.location.global_relative_frame
            turn_data = self.collect_turn_data(
                initial_location,
                current_location,
                target_location,
                velocity,
                heading_change
            )
            
            print(f"Turn progress - Overshoot: {turn_data['overshoot']:.1f}m", end='\r')
            time.sleep(0.1)
        
        print(f"\nTurn completed - Final overshoot: {turn_data['overshoot']:.1f}m")
        return turn_data

    def run_turn_analysis(self, num_tests=10):
        """Run multiple turn tests with random velocities"""
        try:
            # Take off
            self.arm_and_takeoff(30)
            
            print("\nStarting turn analysis tests...")
            for i in range(num_tests):
                print(f"\nTest {i+1}/{num_tests}")
                
                # Return to center position
                self.vehicle.mode = VehicleMode("GUIDED")
                center_location = LocationGlobalRelative(
                    self.vehicle.home_location.lat,
                    self.vehicle.home_location.lon,
                    30
                )
                self.vehicle.simple_goto(center_location)
                time.sleep(10)
                
                # Execute random velocity turn test
                turn_data = self.execute_random_velocity_turn()
                
                # Short pause between tests
                time.sleep(5)
            
            # Return to launch
            print("\nTests completed, returning to launch")
            self.vehicle.mode = VehicleMode("RTL")
            time.sleep(20)
            
            # Save data
            self.save_data()
            
        except Exception as e:
            print(f"Error during analysis: {str(e)}")
        finally:
            self.vehicle.close()

    def save_data(self):
        """Save collected turn analysis data"""
        if not self.turn_data:
            print("No data to save!")
            return
        
        # Save detailed CSV
        df = pd.DataFrame(self.turn_data)
        df.to_csv('turn_analysis_data.csv', index=False)
        print("Turn analysis data saved to turn_analysis_data.csv")
        
        # Save processed JSON for model training
        json_data = [{
            'velocity': data['velocity'],
            'wind_speed': 5.0,  # Simulated wind
            'battery_level': 90.0,  # Simulated battery
            'turn_success': data['turn_success'],
            'overshoot': data['overshoot']
        } for data in self.turn_data]
        
        with open('turn_analysis.json', 'w') as f:
            json.dump(json_data, f, indent=2)
        print("Processed data saved to turn_analysis.json")

def main():
    """Main execution function"""
    print("""
    DRONE TURN PERFORMANCE ANALYZER
    
    This script will:
    1. Take off to 30m altitude
    2. Perform multiple tests with random velocities
    3. For each test:
       - Accelerate to random velocity (5-25 m/s)
       - Execute sudden 90-degree turn
       - Measure turn performance and overshoot
    4. Save detailed analysis data
    
    Make sure Mission Planner SITL is running!
    """)
    
    # Get number of tests
    num_tests = int(input("Enter number of tests to perform (default=10): ") or "10")
    
    # Create analyzer and run tests
    analyzer = DroneTurnAnalyzer()
    if analyzer.connect_drone():
        analyzer.run_turn_analysis(num_tests)

def analyze_results():
    """Analyze the collected turn performance data"""
    df = pd.read_csv('turn_analysis_data.csv')
    
    print("\nTurn Performance Analysis:")
    print("=========================")
    print(f"Total tests conducted: {len(df)}")
    print(f"\nVelocity Statistics:")
    print(f"Average velocity: {df['velocity'].mean():.1f} m/s")
    print(f"Min velocity: {df['velocity'].min():.1f} m/s")
    print(f"Max velocity: {df['velocity'].max():.1f} m/s")
    
    print(f"\nOvershoot Statistics:")
    print(f"Average overshoot: {df['overshoot'].mean():.1f} m")
    print(f"Min overshoot: {df['overshoot'].min():.1f} m")
    print(f"Max overshoot: {df['overshoot'].max():.1f} m")
    
    print(f"\nTurn Success Rate: {(df['turn_success'].mean() * 100):.1f}%")
    
    # Calculate velocity vs overshoot correlation
    correlation = df['velocity'].corr(df['overshoot'])
    print(f"\nVelocity-Overshoot Correlation: {correlation:.3f}")
    
    return df

if __name__ == "__main__":
    main()