class AutonomyController:
    def __init__(self, communicator):
        self.communicator = communicator

    def takeoff(self):
        pass

    def move(self, move_vector):
        pass

    def land(self):
        pass

    def check_location(self, primary, secondary) -> bool:
        pass

    def fly(self):
        while not self.communicator.land.is_set():  # loop until land event is set
            # locks, waits for turn, then gets coordinates
            with self.communicator.autonomy_first_cv:
                while self.communicator.turn != self.communicator.autonomy_first:
                    self.communicator.autonomy_first_cv.wait()
                primary_pos, secondary_pos = self.communicator.get_coordinates()
                self.communicator.change_turn(self.communicator.autonomy_first)

            # check if drone is in bounds
            if not self.check_location(primary_pos, secondary_pos):
                self.land()

            # TODO: get direction from gestures and fly there if possible
            with self.communicator.autonomy_second_cv:
                while self.communicator.turn != self.communicator.autonomy_second:
                    self.communicator.autonomy_second_cv.wait()
                direction = self.communicator.get_gestures()
                # TODO: check if we can fly in that direction
                self.communicator.change_turn(self.communicator.autonomy_second)
            self.move(direction)

        self.land()
