from dronekit import Command, connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil

drone = connect("127.0.0.1:14550", wait_ready=True)

#print(f"Drone Arm Durumu {drone.armed}") 

#print(f"Drone global frame {drone.location.global_frame}")
#print(f"Drone relative global frame {drone.location.global_relative_frame}")
#print(f"İrtifa: {drone.location.global_relative_frame.alt}")

print(f"Drone arm edilebilir mi {drone.is_armable}")

def takeoff(irtifa):

    while drone.is_armable is not True:
        print("Drone arm edilebilir durumda değil")
        time.sleep(1)

    drone.mode = VehicleMode("GUIDED")
    time.sleep(1)
    print(f"{drone.mode}")

    while drone.armed is False:
        print("drone arm ediliyor...")
        drone.armed = True
        time.sleep(1)
    
    drone.simple_takeoff(irtifa)
    
    while drone.location.global_relative_frame.alt < irtifa * 0.9:
        time.sleep(2)
        print("Drone kalkıyor")

takeoff(10)