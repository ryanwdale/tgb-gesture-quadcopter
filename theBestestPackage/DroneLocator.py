import cv2

class DroneLocator:

    # define folder
    def __ini__(self, folder):
        self.folder = folder
        pass

    # this could be done as a seperate thread/process
    def delete_the_weakest(self):
        pass

    def get_most_recent(self):
        pass

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
