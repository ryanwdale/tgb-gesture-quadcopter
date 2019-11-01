import cv2
from threading import Event

class DroneLocator:
    # define folder
    def __init__(self, folder, communicator, stop):
        self.folder = folder
        # connect to camera number 0. connects to web cam on my laptop
        self.video_capture = cv2.VideoCapture(0)
        self.communicator = communicator
        self.stop = stop

    # this could be done as a seperate thread/process
    def delete_the_weakest(self):
        pass

    def get_most_recent(self):
        ret, frame = self.video_capture.read()
        if not ret:
            # couldn't read frame, possibly kill drone
            pass
        return frame

    # todo: dakota and jen
    def locate_drone(self, pic):
        return None, None

    def update(self):
        while not self.stop.is_set():
            picture = self.get_most_recent()
            primary, secondary = self.locate_drone(picture)
            self.communicator.set_coordinates(primary, secondary)


def main():
    dl = DroneLocator("resources")
    dl.update()


if __name__ == "__main__":
    main()
