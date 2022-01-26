


import cv2
import sys
import time
import numpy as np
from multiprocessing import Process
import threading
from threading import Thread

import mediapipe as mp
from slam import SlamRender

class FacialTracking:
    def __init__(self):
        self.classifier = cv2.CascadeClassifier('./model/model.xml')
        self.start = 0
        self.sift = cv2.SIFT_create()

    def run(self, previewName, frame):
        cv2.namedWindow(previewName)
        stop = time.time()
        fps = 1 / (stop - self.start)
        self.start = stop
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        boxes = self.classifier.detectMultiScale(gray)
        for box in boxes:
            x, y, w, h  = box
            x2, y2 = x + w, y + h
            #cv2.rectangle(frame, (x, y), (x2, y2), (0,0,255), 1)
            frame = gray[y: y2, x: x2]
            frame = cv2.resize(frame, (frame.shape[1]*2, frame.shape[0]*2), interpolation = cv2.INTER_AREA)
        kps, des = self.sift.detectAndCompute(frame, None)
        for point in kps:
            x = int(round(point.pt[0]))
            y = int(round(point.pt[1]))
            cv2.circle(frame, (x, y),1,(255,0,0), 1)
        cv2.putText(frame, f'FPS:{int(fps)}', (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow(previewName, frame)

class HandTracking:
    def __init__(self):
        self.start = 0
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(static_image_mode=False,
                              max_num_hands=2,
                              min_detection_confidence=0.5,
                              min_tracking_confidence=0.5)
        self.mpDraw = mp.solutions.drawing_utils

    def run(self, previewName, frame):
        cv2.namedWindow(previewName)
        frame1 = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame1)
        if results.multi_hand_landmarks:
            for handLms in results.multi_hand_landmarks:
                for id, lm in enumerate(handLms.landmark):
                    h, w, c = frame.shape
                    cx, cy = int(lm.x *w), int(lm.y *h)
                    cv2.circle(frame1, (cx, cy), 3, (255,0,255), cv2.FILLED)
                self.mpDraw.draw_landmarks(frame1, handLms, self.mpHands.HAND_CONNECTIONS)
        stop = time.time()
        fps = 1 / (stop - self.start)
        self.start = stop
        cv2.putText(frame1, f'FPS:{int(fps)}', (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow(previewName, frame1)

class TrackingThread(threading.Thread):
    def __init__(self, group=None, target=None, name=None,
                 args=(), kwargs=None, verbose=None):
        super(TrackingThread, self).__init__(group=group, target=target,
                                             name=name)
        self.args = args
        self.kwargs = kwargs
    
    def run(self):
        print("Starting " + self.name)
        #self.camPreview(self.previewName, self.camId)
        while True:
            try:
                if self._target:
                    self._target(self.args[0])
            finally:
                del self._target, self._args, self.kwargs

    def camPreview(self, name, camId):
        cv2.namedWindow(previewName)
        cam = cv2.VideoCapture(camId)
        if cam.isOpened():
            ret, frame = cam.read()
        else:
            ret = False
        while ret:
            cv2.imshow(name, frame)
            key = cv2.waitKey(20)
            if key == 27:
                break
        cv2.destroyWindow(name)

class PlayerTrackers:
    def __init__(self):
        self.head = FacialTracking()
        self.hands = HandTracking()
        self.vid = cv2.VideoCapture(1)

    def run(self):
        while True:
            _, frame = self.vid.read()
            self.head.run("Facial Tracking", frame)
            self.hands.run("Hands Tracking", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.vid.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    i = PlayerTrackers()
    thread1 = Thread(target=i.run, args=())
    thread2 = TrackingThread(target=SlamRender, args=(0, 1 ))
    thread1.start()
    thread2.start()
