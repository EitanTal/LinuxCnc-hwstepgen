from threading import Thread, Event
import time
import usbtest

class globals:
    pass

class MyThread(Thread):
    def __init__(self, event):
        Thread.__init__(self)
        self.stopped = event

    def run(self):
        while not self.stopped.wait(0.5):
            print("my thread")
            # call a function

ep_tx, ep_rx = usbtest.makedev()
globals.tx = ep_tx
globals.rx = ep_rx

stopFlag = Event()
thread = MyThread(stopFlag)
thread.start()
# this will stop the timer
a = input()
stopFlag.set()
