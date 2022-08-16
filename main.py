import cv2
import pyautogui
from cvzone.HandTrackingModule import HandDetector
import time
import numpy as np
import pyautogui as pag
def main():

    frameR=100
    smoothening = 7
    wCam,hCam=640,480
    plocX, plocY = 0, 0
    clocX, clocY = 0, 0
    cap=cv2.VideoCapture(0)
    cap.set(3,wCam)
    cap.set(4,hCam)
    detector= HandDetector(detectionCon=0.8, maxHands=1)
    pTime=0
    wScr,hScr=1920,1080
    while True:
        ret, frame = cap.read()
        frame = cv2.flip(frame, 1)
        hands,frame=detector.findHands(frame,flipType=False)
        if hands:
            hand1=hands[0]
            lmList = hands[0]['lmList']
            x1,y1,_=lmList[8]
            # print(y1)
            x2,y2=lmList[12][1:]
            fingers=detector.fingersUp(hand1)
            # print(fingers)
            if fingers[1]==1 and fingers[2]==0:
                x3 = np.interp(x1, (frameR, wCam - frameR), (0, wScr))
                y3 = np.interp(y1, (frameR, hCam - frameR), (0, hScr))
                clocX = plocX + (x3 - plocX) / smoothening
                clocY = plocY + (y3 - plocY) / smoothening
                pag.moveTo(wScr - clocX, hScr-clocY)
                cv2.circle(frame, (x1, y1), 15, (255, 0, 255), cv2.FILLED)
                plocX, plocY = clocX, clocY




            if fingers[1]==1 and fingers[2]==1:
                lent, _, _ = detector.findDistance(lmList[8], lmList[12], frame)
                if lent<40:
                    pag.click()
                    # print("yes")

        cTime=time.time()
        fps=1/(cTime-pTime)
        pTime=cTime
        cv2.imshow("Image", frame)
        key=cv2.waitKey(1)
        if key==27:
                 break


if __name__ == "__main__":
    main()
