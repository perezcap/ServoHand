
import time
from dynamixel_sdk import *  # Import the Dynamixel SDK library

# Control table addresses for XM430-W210 Dynamixel
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_VELOCITY = 128
PROTOCOL_VERSION = 2.0

# Default setting
DXL_IDs = [10,11,12,20,21,22,30,31,32]  # IDs 
BAUDRATE = 1000000  # 1Mbps
DEVICENAME = 'COM7'  # Remember to change devicename whether it is Windows, Linux, Mac - 'COM7' for windows

# Initialize PortHandler instance
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()

# Enable Dynamixel Torque
def enable_torque(portHandler, packetHandler, DXL_ID):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 1)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Torque enabled for ID:", DXL_ID)

# Set goal position
def set_goal_position(portHandler, packetHandler, DXL_ID, goal_position):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, goal_position)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Goal position set for ID:", DXL_ID)

# Set home position 
def set_home_position(portHandler, packetHandler, DXL_ID, home_position):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, home_position)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Home position set for ID:", DXL_ID)


# Read present position
def read_present_position(portHandler, packetHandler, DXL_ID):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        # Convert the raw position value to degrees
        present_position_degrees = dxl_present_position * 0.088
        #print("Present position:", present_position_degrees, "degrees")
        return present_position_degrees

# Read present velocity
def read_present_velocity(portHandler, packetHandler, DXL_ID):
    dxl_present_velocity, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_VELOCITY)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        # Convert the raw velocity value to rpm
        present_velocity_rpm = dxl_present_velocity * 0.229 / 1023
        #print("Present velocity:", present_velocity_rpm, "rpm")
        return present_velocity_rpm


# Enable torque for all motors
for DXL_ID in [10,11,12,20,21,22,30,31,32]:
    enable_torque(portHandler, packetHandler, DXL_ID)

# all motors in vertical position
home_positions = {10:1024, 11:2200, 12:2500, 20: 1024, 21: 2200, 22: 2500, 30: 1024, 31: 2200, 32: 2500}

# Define goal positions for each motor (in Dynamixel units)
goal_positions = {10: 1024, 11: 1280, 12: 2550, 20: 1024, 21: 1280, 22: 2550, 30: 1024, 31: 1280, 32: 2550}

# Set goal positions for each motor
for DXL_ID, goal_position in goal_positions.items():
    set_goal_position(portHandler, packetHandler, DXL_ID, goal_position)

# Read and print present position and velocity for each motor
for DXL_ID in [10,11,12,20,21,22,30,31,32]:
    present_position = read_present_position(portHandler, packetHandler, DXL_ID)
    present_velocity = read_present_velocity(portHandler, packetHandler, DXL_ID)
    print("Motor ID:", DXL_ID)
    print("Present position:", present_position, "degrees")
    print("Present velocity:", present_velocity, "rpm")

time.sleep(1)

#Set motors to home position
for DXL_ID, home_position in home_positions.items():
    set_home_position(portHandler, packetHandler, DXL_ID, home_position)
    
# Close port
portHandler.closePort()

  