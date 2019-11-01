from theBestestPackage.DroneLocator import DroneLocator
from threading import Thread, Event
from typing import List, Tuple


class CommunicationController:
    def __init__(self):
        # movement vector from gestures
        self.move_vector = []
        self.land = Event()      # drone lands when set

        # boundary information from coordinates
        self.primary_pos = []    # location from primary camera
        self.secondary_pos = []  # location from secondary camera

        self.stop_updating = Event()  # stops DroneLocator thread when set
        self.drone_locator = DroneLocator("resources", self, self.stop_updating)
        self.coordinates_thread = Thread(target=self.drone_locator.update)

        # TODO: add gestures and autonomy objects and threads

    def start(self):
        """
        starts threads
        :return: None
        """

        self.coordinates_thread.start()
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
