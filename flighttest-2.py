import airsimneurips
import time
import numpy as np
import heapq
from scipy.spatial.transform import Rotation as R
from airsimneurips import MultirotorClient

# Connect to server with drone controls
client = MultirotorClient()
client.confirmConnection()
print('Connection confirmed')

client.enableApiControl(vehicle_name="drone_1")
client.arm(vehicle_name="drone_1")

# Retrieve all objects and filter out gates
sceneObjects = client.simListSceneObjects()
gatesObjects = sorted([x for x in sceneObjects if "Gate" in x])
print("Detected gates:", gatesObjects)

# Get gate positions
gatePositions = [client.simGetObjectPose(gate).position for gate in gatesObjects]

# Function to extract yaw from quaternion
def get_yaw_from_quaternion(quaternion):
    r = R.from_quat([quaternion.x_val, quaternion.y_val, quaternion.z_val, quaternion.w_val])
    euler = r.as_euler('xyz', degrees=True)
    return euler[2]  # Yaw (rotation around Z-axis)

# A* Search Algorithm with Quaternion-Based Optimization
def heuristic(a, b):
    return np.linalg.norm([b.x_val - a.x_val, b.y_val - a.y_val, b.z_val - a.z_val])

def astar_path(start, goal, waypoints, current_quaternion):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {point: float('inf') for point in waypoints}
    g_score[start] = 0
    f_score = {point: float('inf') for point in waypoints}
    f_score[start] = heuristic(start, goal)

    current_yaw = get_yaw_from_quaternion(current_quaternion)

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        for neighbor in waypoints:
            if neighbor == current:
                continue
            tentative_g_score = g_score[current] + heuristic(current, neighbor)

            move_yaw = np.arctan2(neighbor.y_val - current.y_val, neighbor.x_val - current.x_val) * (180 / np.pi)
            yaw_diff = abs(move_yaw - current_yaw) % 360
            yaw_penalty = yaw_diff / 90
            tentative_g_score += yaw_penalty

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None  # No valid path found

# Start race and takeoff
client.simStartRace()
client.takeoffAsync(vehicle_name="drone_1").join()
client.moveToZAsync(vehicle_name="drone_1", z=-2.0, velocity=8.0).join()

# Plan full path through gates
pose = client.simGetVehiclePose()
current_quaternion = pose.orientation

# Full A* path planning between gates
full_path = [gatePositions[0]]
for i in range(len(gatePositions) - 1):
    segment = astar_path(gatePositions[i], gatePositions[i + 1], gatePositions, current_quaternion)
    if segment:
        full_path.extend(segment[1:])

# Fly to each waypoint sequentially
velocity = 10.0
thresholdDistance = 0.75
for waypoint in full_path:
    while True:
        current_pose = client.simGetVehiclePose(vehicle_name="drone_1").position
        distance = heuristic(current_pose, waypoint)

        if distance <= thresholdDistance:
            print(f"Reached waypoint at: ({waypoint.x_val:.2f}, {waypoint.y_val:.2f}, {waypoint.z_val:.2f})")
            break

        client.moveToPositionAsync(
            waypoint.x_val, waypoint.y_val, waypoint.z_val,
            velocity, vehicle_name="drone_1"
        )
        time.sleep(0.05)

# Land the drone and disarm everything
client.landAsync(vehicle_name="drone_1").join()
client.disarm(vehicle_name="drone_1")
client.disableApiControl(vehicle_name="drone_1")
