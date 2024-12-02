import airsimneurips


#connect to server with drone controls
client = airsimneurips.MultirotorClient()
client.confirmConnection()
print('Connection confirmed')

"""
To load the level first comment out the code below and run the exe file
Once the level is loaded comment out the simLoadLevel code and uncomment your code
Run the python script again and you should see the drone takeoff
"""

client.simLoadLevel('Soccer_Field_Easy')

# client.enableApiControl(vehicle_name="drone_1")
# client.arm(vehicle_name="drone_1")

# # Async methods returns Future. Call join() to wait for task to complete.
# client.takeoffAsync(vehicle_name="drone_1").join()

# Code goes here to move the drone

#client.moveToZAsync(vehicle_name="drone_1", z= 2.05, velocity=4.5).join()   # Move drone to an altitude to avoid obstacles

# Retrieve all objects and filter out gates
sceneObjects = client.simListSceneObjects()
gatesObjects = sorted([x for x in sceneObjects if "Gate" in x])
print(gatesObjects)

# Gate position lists
#xPos = []
#yPos = []
#zPos = []

# Append the list with the gates
gatePositions = [client.simGetObjectPose(gate) for gate in gatesObjects]
"""
for gate in gatesObjects:
    gatePose = client.simGetObjectPose(gate).position
    gatePositions.append(gatePose)
    """

#firstGate = gatePositions[0]
client.simStartRace()
client.takeoffAsync(vehicle_name="drone_1").join()
#client.moveToZAsync(-2, velocity=5, vehicle_name="drone_1").join()

client.moveToPositionAsync(gatePositions[0].position.x_val, gatePositions[0].position.y_val, gatePositions[0].position.z_val, velocity=5.5, vehicle_name="drone_1").join()  # Reach 2m altitude immediately

velocity = 5.5
#approachVelocity = 1
#intermediateDist = 0.5
thresholdDistance = 0.5
#timeoutDuration = 10

for gate_position in gatePositions:
    while True:
        # Get current drone position
        current_pose = client.simGetVehiclePose(vehicle_name="drone_1").position
        distance = np.sqrt(
            (gate_position.position.x_val - current_pose.x_val)**2 +
            (gate_position.position.y_val - current_pose.y_val)**2 +
            (gate_position.position.z_val - current_pose.z_val)**2
        )
        
        # Check if the drone is close enough to the gate
        if distance <= thresholdDistance:
            print(f"Reached gate at position: {gate_position}")
            break  # Proceed to the next gate

        # Move toward the current gate position
        client.moveToPositionAsync(
            gate_position.position.x_val, gate_position.position.y_val, gate_position.position.z_val,
            velocity, vehicle_name="drone_1"
        )
"""
for gate_position in gatePositions:
    start_time = time.time()  # Track the start time for the timeout
    reached_gate = False  # Flag to track if the current gate is reached

    # Get current drone position
    current_pose = client.simGetVehiclePose(vehicle_name="drone_1").position
        #current_position = current_pose.position
        #current_orientation = current_pose.orientation

    direction_to_gate = np.array([
        gate_position.position.x_val - current_pose.x_val,
        gate_position.position.y_val - current_pose.y_val,
        gate_position.position.z_val - current_pose.z_val
    ])
    direction_to_gate = direction_to_gate / np.linalg.norm(direction_to_gate)
    intermediate_waypoint = np.array([
        gate_position.position.x_val - intermediateDist * direction_to_gate[0],
        gate_position.position.y_val - intermediateDist * direction_to_gate[1],
        gate_position.position.z_val - intermediateDist * direction_to_gate[2]
    ])

    client.moveToPositionAsync(
        intermediate_waypoint[0], intermediate_waypoint[1], intermediate_waypoint[2],
        velocity, vehicle_name="drone_1"
    ).join()


    while not reached_gate:

        current_pose = client.simGetVehiclePose(vehicle_name="drone_1").position

        distance = np.sqrt(
            (gate_position.position.x_val - current_pose.x_val)**2 +
            (gate_position.position.y_val - current_pose.y_val)**2 +
            (gate_position.position.z_val - current_pose.z_val)**2
        )
        
        # Check if the drone is close enough to the gate
        if distance <= thresholdDistance:
            print(f"Reached gate at position: {gate_position}")
            reached_gate = True  # Move on to the next gate
            time.sleep(0.5)
            break
        
        
        # Check for timeout condition
        if time.time() - start_time > timeoutDuration:
            print("Timeout reached, moving to the next gate")
            #thresholdDistance = 2.0  # Temporarily increase threshold to avoid re-targeting this gate
            reached_gate = True
            break
        
        direction = np.array([
            gate_position.position.x_val - current_pose.x_val,
            gate_position.position.y_val - current_pose.y_val,
            gate_position.position.z_val - current_pose.z_val
        ])
        direction = direction / np.linalg.norm(direction)  # Normalize direction

        # Incremental target position
        incremental_target = np.array([
            current_pose.x_val + intermediateDist * direction[0],
            current_pose.y_val + intermediateDist * direction[1],
            current_pose.z_val + intermediateDist * direction[2]
        ])

        currentVelocity = approachVelocity if distance <= 3 else velocity
        
        # Move toward the current gate position
        
        yaw_to_gate = np.arctan2(
            gate_position.position.y_val - current_pose.y_val,
            gate_position.position.x_val - current_pose.x_val
        )
        
        client.moveToPositionAsync(
            gate_position.position.x_val, gate_position.position.y_val, gate_position.position.z_val,
            currentVelocity, vehicle_name="drone_1"
        )
        #client.moveToYawAsync(np.degrees(yaw_to_gate), vehicle_name="drone_1").join()
        time.sleep(0.1)  # Small delay to update position frequently

    # Reset threshold for the next gate
    #threshold_distance = 1.0
"""

# Land the drone and disarm everything
client.landAsync(vehicle_name="drone_1").join()
client.disarm(vehicle_name="drone_1")
client.disableApiControl(vehicle_name="drone_1")

