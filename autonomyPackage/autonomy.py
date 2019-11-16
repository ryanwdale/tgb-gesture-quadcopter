from threading import Event, Thread
from multiprocessing import Pipe
from dronekit import connect


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

    def fly(self, primary_pos, secondary_pos, directional_vector):
        currentlyFlying = True
        while not self.land_event.is_set():
            if not self.check_location(primary_pos, secondary_pos):
                self.land()
            self.move(directional_vector)
        self.land()
        currentlyFlying = False
