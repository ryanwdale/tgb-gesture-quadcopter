from threading import Event
from dronekit import connect

class AutonomyController:
    def takeoff(self, altitude):

        vehicle = connect(connection_string, wait_ready=True)
        while vehicle.is_armable() == True:
           vehicle.mode = dronekit.VehicleMode("GUIDED")
           vehicle.armed == True
           vehicle.simple_takeoff(altitude)


    def move(self, move_vector):
        pass
     #comment

    def land(self):

        #check location
        vehicle.mode = VehicleMode("LAND")
        #when altitude reaches 0, send message 'landed'
        vehicle.close()
        stil.stop()

    def check_location(self, primary, secondary) -> bool:
        pass

    def fly(self, primary_pos, secondary_pos, directional_vector):
        while not self.land_event.is_set():
            if not self.check_location(primary_pos, secondary_pos):
                self.land()
            self.move(directional_vector)
        self.land()
