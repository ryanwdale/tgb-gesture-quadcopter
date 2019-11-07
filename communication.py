from theBestestPackage.DroneLocator import DroneLocator
from autonomyPackage.autonomy import AutonomyController
from multiprocessing import Process, Event, Array
from typing import List, Tuple


class CommunicationController:
    def __init__(self):
        # movement vector from gestures
        move_vector = Array('d', [0, 0, 0])  # type double

        # boundary information from coordinates
        primary_pos = Array('i', [0, 0, 0])  # location from primary camera, type int
        secondary_pos = Array('i', [0, 0, 0])  # location from secondary camera, type int

        # coordinates stuff
        self.stop_updating = Event()  # stops DroneLocator thread when set
        self.drone_locator = DroneLocator("resources")
        self.coordinates_process = Process(target=self.drone_locator.update,
                                           args=(primary_pos,
                                                 secondary_pos,
                                                 move_vector,
                                                 self.land))

        # autonomy stuff
        self.land = Event()  # drone lands when set
        self.autonomy_controller = AutonomyController()
        self.autonomy_process = Process(target=self.autonomy_controller.fly,
                                        args=(primary_pos,
                                              secondary_pos,
                                              self.land))

        # TODO: add gestures and autonomy objects and threads

    def start(self):
        """
        starts threads
        :return: None
        """

        self.coordinates_process.start()
        self.autonomy_process.start()

        # TODO: create gestures and autonomy threads

    def stop(self):
        """
        stops threads

        :return: None
        """
        self.stop_updating.set()
        self.land.set()

        self.coordinates_process.join()
        self.autonomy_process.join()
        # TODO: set stop events for other threads
