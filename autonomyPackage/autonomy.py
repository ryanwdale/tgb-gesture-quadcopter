from threading import Event, Thread
from multiprocessing import Pipe
from dronekit import connect

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.

    MAV_FRAME_BODY_OFFSET_NED - makes velocity component relative to drone's current heading (Currently used)
                                ex. North = front of drone, South = back of drone, East = right, West = left
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
        
        
class AutonomyController:
    def __init__(self):
        self.commThread = Thread(target=self.receiveCommand) 
        self.receiver = Pipe()
        self.currentlyFlying = False  # whether the drone is currently executing a flight command
        self.command = None

    def main(self):
        # start the threads, then do cleanup once flightCtrl terminates
        commThread.start()
        self.flightCtrl()

        receiver.close()
        commThread.join()

    def takeoff(self, altitude):

        vehicle = connect(connection_string, wait_ready=True)
        while vehicle.is_armable() == True:
           vehicle.mode = dronekit.VehicleMode("GUIDED")
           vehicle.armed == True
           vehicle.simple_takeoff(altitude)

    def receiveCommand(self):
        # receive the flight commands from Jin
        while True:
            res = receiver.recv()
            if currentlyFlying:
                continue  # Done is currently executing last instruction, ignore incoming instructions
            else
                command = res

    
    def move(self, move_vector):
        pass

    def land(self):

        #check location
        vehicle.mode = VehicleMode("LAND")
        #when altitude reaches 0, send message 'landed'
        vehicle.close()
        stil.stop()

    def flightCtrl(self):
        # The main decision process for autonomy.
        # This function should terminate when the process is ready to terminate.
        pass

    def check_location(self, primary, secondary) -> bool:
        pass
        #calculate location and check if it is safe to move to that location

        
    """
    x > 0 => fly North
    x < 0 => fly South
    y > 0 => fly East
    y < 0 => fly West
    z < 0 => ascend
    z > 0 => descend
    """
    def fly(self, x,y,z, FLIGHTDURATION=5):
        currentlyFlying = True
        if self.check_location:
            send_ned_velocity(x, y, z, FLIGHTDURATION)
            send_ned_velocity(0, 0, 0, 1)
        Else:
            print("Invalid movement\n")
        currentlyFlying = False
