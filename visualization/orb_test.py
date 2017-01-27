import cv2
import glob
import time

class Imreader():
  def __init__(self, directory, suffix=".jpg"):
    self.filenames = glob.glob(directory + "*" + suffix)

  def get_next_image(self):
    index = 0
    while index < len(self.filenames):
      yield cv2.imread(self.filenames[index])
      index += 1

  def get_len(self):
    return len(self.filenames)

def main():
  #reader = Imreader('../tests/ims')
  imfiles = glob.glob('../tests/ims/*.jpg')
  ims = [cv2.imread(im) for im in imfiles]
  print "fetched: ", len(ims)
  orb = cv2.ORB_create()

  now = time.clock()
  for img in ims:
      kp = orb.detect(img,None)
      kp, des = orb.compute(img, kp)
  done = time.clock()

  print "rate: ", str((done - now)/len(ims)), "per image"

if __name__ == "__main__":
  main()
