import numpy as np
import cv2
from datetime import datetime

def dataset_camera():
  cap = cv2.VideoCapture(0)

  # Define the codec and create VideoWriter object
  # # fourcc = cv2.VideoWriter_fourcc(*'XVID')
  # # out = cv2.VideoWriter('output.avi',fourcc, 20.0, (1280,720))

  # Verify input shape
  width = cap.get(3)
  height = cap.get(4)
  fps = cap.get(5)
  print(width, height, fps)

  while(cap.isOpened()):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Write the frame in colour
    timestamp = datetime.timestamp(datetime.now())
    filename = str(timestamp) + ".jpg"
    cv2.imwrite("raw-images/" + "colour-" + filename, frame) 

    # Write the frame in greyscale
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imwrite("raw-images/" + "grey-" + filename, grey) 

    # Display the resulting frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

  # Release everything if job is finished
  cv2.destroyAllWindows()
  cap.release()

def dataset_make_more_samples(imgFilename):
  image = cv2.imread(imgFilename)

  # Create grey images
  grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
  cv2.imwrite("grey-" + imgFilename)

  # Create HGR images
  hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
  cv2.imwrite("hsv-" + imgFilename)

  # Numerous transformations

if __name__ == '__main__':
  dataset_camera()
