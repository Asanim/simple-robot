import time
import json
import greengrasssdk
import threading
import math

# Create Greengrass core SDK client
client = greengrasssdk.client('iot-data')

# Define topics
path_planning_topic = 'robot/path_planning'
position_topic = 'robot/position'
twist_velocity_topic = 'robot/twist_velocity'
control_topic = 'robot/control_twist'

# Robot's current position and velocity (simulated)
robot_position = {'x': 0.0, 'y': 0.0}
robot_velocity = {'linear': 0.0, 'angular': 0.0}

# Function to publish data to Greengrass MQTT topics
def publish_data():
    while True:
        # Path planning (just simulated here)
        path_planning = {'goal': {'x': 5.0, 'y': 5.0}, 'current_position': robot_position}

        # Publish path planning data
        client.publish(
            topic=path_planning_topic,
            payload=json.dumps(path_planning)
        )

        # Publish robot position (x, y)
        client.publish(
            topic=position_topic,
            payload=json.dumps(robot_position)
        )

        # Publish twist velocity (linear and angular)
        twist_velocity = {'linear': robot_velocity['linear'], 'angular': robot_velocity['angular']}
        client.publish(
            topic=twist_velocity_topic,
            payload=json.dumps(twist_velocity)
        )

        time.sleep(1)  # Publish every second


# Function to handle incoming control messages (Twist commands)
def control_robot(payload):
    try:
        # Parse the incoming twist command (JSON format)
        control_data = json.loads(payload)
        linear = control_data.get('linear', 0.0)
        angular = control_data.get('angular', 0.0)

        # Update robot's velocity (you can replace this with real control logic)
        robot_velocity['linear'] = linear
        robot_velocity['angular'] = angular

        # Simulate robot movement based on received twist command
        robot_position['x'] += robot_velocity['linear'] * math.cos(robot_velocity['angular'])
        robot_position['y'] += robot_velocity['linear'] * math.sin(robot_velocity['angular'])

        print(f"Robot Control: Linear Velocity: {linear}, Angular Velocity: {angular}")
        print(f"New Robot Position: {robot_position}")

    except Exception as e:
        print(f"Error processing control message: {e}")

# Subscribe to the control twist topic
def subscribe_to_control_topic():
    while True:
        try:
            # This should be replaced by the actual Greengrass SDK method for subscribing to the topic
            response = client.subscribe(topic=control_topic)
            if response:
                # Pass the payload to control_robot function when a message is received
                control_robot(response['payload'])
        except Exception as e:
            print(f"Error subscribing to control topic: {e}")
        time.sleep(1)


# Main function to start both publishing and subscribing
def main():
    # Start publishing data in a separate thread
    publish_thread = threading.Thread(target=publish_data)
    publish_thread.daemon = True
    publish_thread.start()

    # Start subscribing to control topic
    subscribe_to_control_topic()


if __name__ == '__main__':
    main()
