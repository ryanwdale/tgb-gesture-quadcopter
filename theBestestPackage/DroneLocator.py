import cv2

class DroneLocator:

    # define folder
    def __init__(self, folder):
        self.folder = folder
        # connect to camera number 0. connects to web cam on my laptop
        self.video_capture = cv2.VideoCapture(0)
        pass

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
        pass

    def update(self):
        picture = self.get_most_recent()
        self.locate_drone(picture)


def main():
    dl = DroneLocator("resources")
    # for now has to be manually stopped
    while True:
        dl.update()


if __name__ == "__main__":
    main()
