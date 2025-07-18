import cv2
import os
from os import listdir
from os.path import isfile, join
import numpy as np
import time

name = "new_printed_ammunition"
output_dir = "/Vision_dev/src/scripts/imagens5/" + name + "/"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

video_capture = cv2.VideoCapture(4)

resolucao_x = 640
resolucao_y= 480

video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, resolucao_x)
video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, resolucao_y)


count = 0
while True:
    ok, frame = video_capture.read()
    if ok:
        #frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        count += 1
        if count <= 100:
            cv2.imwrite(output_dir + name + str(count) + ".png", frame)
            cv2.putText(frame, "Got " + str(count) + " images", (20,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)
        else:
                cv2.putText(frame, "Done", (20,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)
        
        time.sleep(0.1)
        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == 27:
            break  # Exit loop if 'Esc' key is pressed
       
cv2.destroyAllWindows()
video_capture.release()
