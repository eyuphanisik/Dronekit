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

# loc = LocationGlobalRelative(-35.36233482, 149.16509180, 20)

# drone.simple_goto(loc)

def gorev_ekle():
    global komut
    komut = drone.commands
    komut.clear()
    time.sleep(1)

    #TAKEOFF
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0 ,0, 0, 0, 0, 0, 0, 0, 10))

    #WAYPOINT
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0 ,0, 0, 0, 0, 0, -35.36249121, 149.16513285 , 20))
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0 ,0, 0, 0, 0, 0, -35.36298474, 149.16655027 , 10))

    #RTL
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0 ,0, 0, 0, 0, 0, 0, 0, 0))

    #DOĞRULAMA
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0 ,0, 0, 0, 0, 0, 0, 0, 0))

    komut.upload()
    print("Görevler yükleniyor...")

def gorev_yap():
    drone.mode = VehicleMode("AUTO")
    komut.next = 0

    while True:
        next_waypoint = komut.next

        if next_waypoint == 4:
            print("Görev bitti")
            break

        print(f"Sıradaki komut {next_waypoint}")
        time.sleep(1)
        

takeoff(10)

gorev_ekle()
gorev_yap()