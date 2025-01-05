a = []
last_a = 0
import time, threading
def foo():
    global a
    global last_a
    #threading.Timer(0.01, foo).start()
    #val = int(time.time() * 10000) % 10000
    
    dt = 0
    while (dt < 10):
        val = int(time.time() * 10000)
        dt = val - last_a
    last_a = val
    a += [dt]
    if (len(a) >= 25):
        print (a)
        a = []
    threading.Timer(0.00075, foo).start()
    
    

foo()

