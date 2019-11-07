from threading import Event


class AutonomyController:
    def takeoff(self):
        pass

    def move(self, move_vector):
        pass

    def land(self):
        pass

    def check_location(self, primary, secondary) -> bool:
        pass

    def fly(self, primary_pos, secondary_pos, directional_vector):
        while not self.land_event.is_set():
            if not self.check_location(primary_pos, secondary_pos):
                self.land()
            self.move(directional_vector)
        self.land()
