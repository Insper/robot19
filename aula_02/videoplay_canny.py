#-*- coding:utf-8 -*-
import cv2

cap = cv2.VideoCapture('hall_box_battery_1024.mp4')

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    dst = cv2.Canny(gray, 50, 200) # aplica o detector de bordas de Canny à imagem src


    # Display the resulting frame
    cv2.imshow('frame',dst)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
