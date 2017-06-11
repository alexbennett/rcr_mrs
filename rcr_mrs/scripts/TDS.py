#!/usr/bin/env python

from TDSdev import tdsmain
import threading
import time

class TDS:
    def __init__(self):
        # Setup thread event
        self._done = threading.Event()

    def start(self):
        # Terminate previous
        self.terminate()

        # Reset thread event status
        self._done.clear()

        # Spawn thread
        try:
            # Create thread
            self._detection_thread = threading.Thread(target=self.do_detection, args=())

            # Start thread
            self._detection_thread.start()

            # Allow thread to start
            time.sleep(1)
        except:
            fault("Error: Unable to start thread")

    def do_detection(self):
        # Create global tracking variable
        global look_for

        # Create global altitude
        global altitude

        # Set tracking variable
        look_for = 'bry'

        # Set altitude
        altitude = 160

        # Perform detection while there are still targets left in the tracking variable
        while(len(look_for) > 0):
            # Call object detect
            found = tdsmain.objectDetect(altitude, look_for)

            # Check for what is found
            for target in found:
                look_for = look_for.replace(target, "")

    def terminate(self):
        # Terminate the thread by setting the done event
        self._done.set()

        # Allow the system time to respond
        time.sleep(0.1)
