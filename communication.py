from theBestestPackage.DroneLocator import DroneLocator
from autonomyPackage.autonomy import AutonomyController
from threading import Thread, Event, Condition, Lock
from typing import List, Tuple


class CommunicationController:

    coordinates, autonomy_first, gestures, autonomy_second = range(4)

    def __init__(self):

        # movement vector from gestures
        self.move_vector = []

        # coordinates stuff
        self.stop_updating = Event()  # stops DroneLocator thread when set
        self.drone_locator = DroneLocator("resources", self)
        self.coordinates_thread = Thread(target=self.drone_locator.update)

        # boundary information from coordinates
        self.primary_pos = []  # location from primary camera
        self.secondary_pos = []  # location from secondary camera

        # autonomy stuff
        self.land = Event()  # drone lands when set
        self.autonomy_controller = AutonomyController(self)
        self.autonomy_thread = Thread(target=self.autonomy_controller.fly)

        # conditional variable stuff
        lock = Lock()
        self.turn = self.coordinates

        # conditional variables to ensure threads aren't being starved
        # threads take turns one by one
        # current turns are as follows:
        # 1) coordinates sets current position
        # 2) autonomy gets that position
        # 3) gestures sets direction
        # 4) autonomy gets direction
        cond_vars = [self.coordinates_cv, self.autonomy_first_cv, self.gestures_cv, self.autonomy_second_cv]
        self.turn_map = {i: cond_vars[i] for i in range(4)}
        self.autonomy_first_cv = Condition(lock=lock)
        self.coordinates_cv = Condition(lock=lock)
        self.autonomy_second_cv = Condition(lock=lock)
        self.gestures_cv = Condition(lock=lock)



        # TODO: add gestures and autonomy objects and threads

    def start(self):
        """
        starts threads
        :return: None
        """

        self.coordinates_thread.start()
        self.autonomy_thread.start()

        # TODO: create gestures and autonomy threads

    def stop(self):
        """
        stops threads

        :return: None
        """
        self.stop_updating.set()

        # TODO: set stop events for other threads

    def set_coordinates(self, primary: List[int], secondary: List[int]):
        """
        Called by the DroneLocator to update the Drones location

        :param primary: Coordinate position of drone from perspective of primary camera
        :param secondary: Coordinate position of drone from perspective of secondary camera
        :return: None
        """

        self.primary_pos = primary
        self.secondary_pos = secondary

    def set_gestures(self, move: List[float], do_land: bool):
        """
        Called by gestures object to update the direction the drone should move

        :param move: Directional vector, indicates direction the drone should move
        :param do_land: Drone should land when True
        :return: None
        """
        if do_land:
            self.land.set()
        self.move_vector = move

    def get_coordinates(self) -> Tuple[List[int], List[int]]:
        return self.primary_pos, self.secondary_pos

    def get_gestures(self) -> List[float]:
        return self.move_vector

    def change_turn(self, turn):
        self.turn = (turn + 1) % 4
        self.turn_map[turn].notify()
