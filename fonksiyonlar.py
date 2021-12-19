from dronekit import Command, connect
from pymavlink import mavutil
import time

drone = connect("127.0.0.1:14550", wait_ready=True)

def goto_position_target_local_ned(north, east, down): #default konumdan kaç metre hangi yönde gideceğini parametre alır
    """
    Send SET_POSITION_TARGET_LOCAL_NED command to request the drone fly to a specified
    location in the North, East, Down frame.
    """
    msg = drone.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down,
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to drone
    drone.send_mavlink(msg)

def goto_position_target_relative_ned(north, east, down): #kendi konumdan kaç metre hangi yönde gideceğini parametre alır
    """
    Send SET_POSITION_TARGET_LOCAL_NED command to request the drone fly to a specified
    location in the North, East, Down frame.
    """
    msg = drone.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down,
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to drone
    drone.send_mavlink(msg)

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration): # hangi eksende ne kadar hızla ve ne kadar süre devam ediceğini parametre alır
 
    msg = drone.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to drone on 1 Hz cycle
    for x in range(0,duration):
        drone.send_mavlink(msg)
        time.sleep(1)

def condition_yaw(heading, relative=False): # dronenun yönünü değiştirir

    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = drone.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to drone
    drone.send_mavlink(msg)

#goto_position_target_local_ned(51, 51, -5)
#send_ned_velocity(5, 0, -1, 5)
condition_yaw(180, True) #false olursa kuzeye göre kendini döndürür true olursa aracın yönüne göre
#goto_position_target_relative_ned(50, 10, 4)