
class Boundary:

    WIDTH = 800
    HEIGHT = 600

    def __ini__(self, quad_width, quad_height):
        self.quadWidth = quad_width
        self.quadHeight = quad_height
        self.pixelWidth = self.WIDTH/quad_width
        self.pixelHeight = self.HEIGHT/quad_height

    def initialize(self):
        pass

    # box needs to be decided on based on openCV
    def get_quad(self, box):
        pass
