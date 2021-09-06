# timer.py

import time

class Timer:
    def __init__(self):
        self._start_time = None

    def start(self):
        """Start a new timer"""
        if self._start_time is None:
            self._start_time = time.perf_counter()

    def stop(self):
        """Stop the timer"""
        self._start_time = None

    def reset(self):
        """Reset the timer"""
        self._start_time = time.perf_counter()

    def elapsed(self):
        """Get elapsed time"""
        if self._start_time is not None: 
            elapsed_time = time.perf_counter() - self._start_time
            return elapsed_time
        else:
            return 0