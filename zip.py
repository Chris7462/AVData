import cv2
import glob
import os

if __name__ == '__main__':
  os.chdir('./FR')
  file_name = glob.glob("*.png")
  os.chdir('../')
  if not os.path.exists("./zip/"):
    os.mkdir("./zip/")
  os.chdir('./zip')
  folder = ['FR','SR','SL','RR','RL']
  for fo in folder:
    if not os.path.exists('./' + fo + '/'):
      os.mkdir('./' + fo + '/')
    for fi in file_name:
      img = cv2.imread('./' + fo + '/' + fi)
      img_z = cv2.resize(img, (414,215), interpolation = cv2.INTER_CUBIC)
      cv2.imwrite('./zip/' + fo + '/' + fi, img_z)
