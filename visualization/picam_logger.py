import cv2
import picamera

import time

def main():
  camera = picamera.PiCamera()
  #let the gain and awb settle
  time.sleep(2)
  #set gains/ss/etc for consistent images
  camera.shutter_speed = camera.exposure_speed
  camera.exposure_mode = 'off'
  g = camera.awb_gains
  camera.awb_mode = 'off'
  camera.awb_gains = g

  for i in xrange(80):
    camera.capture('../tests/ims/' + ("%03d" % i) + ".jpg")
    print("captured new image", i)


if __name__ == "__main__":
  main()
