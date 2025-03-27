import time
import py_trees

class CustomTimer(py_trees.behaviour.Behaviour):
    def __init__(self, name, duration):
        super().__init__(name)
        self.duration = duration  # duration in seconds
        self.start_time = None
        self.completed = False

    def update(self):
        # Start the timer when it's first ticked
        if self.start_time is None:
            self.start_time = time.time()  # Record the start time when timer starts

        # Calculate elapsed time
        elapsed_time = time.time() - self.start_time

        # If the elapsed time has exceeded the duration, return success
        if elapsed_time >= self.duration:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE