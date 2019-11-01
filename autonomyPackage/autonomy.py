from threading import Event


class AutonomyController:
    def __init__(self, communicator, land: Event):
        self.communicator = communicator
        self.land_event = land

    def takeoff(self):
        pass

    def move(self, move_vector):
        pass

    def land(self):
        pass

    def check_location(self, primary, secondary) -> bool:
        pass

    def fly(self):
        while not self.land_event.is_set():
            primary_pos, secondary_pos = self.communicator.get_coordinates()
            if not self.check_location(primary_pos, secondary_pos):
                self.land()

        self.land()
