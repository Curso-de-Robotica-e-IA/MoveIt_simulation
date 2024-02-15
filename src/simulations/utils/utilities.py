from contextlib import contextmanager
from datetime import datetime


@contextmanager
def measure_duration(*args):
    start_time = datetime.now()
    print("starting time:", start_time)
    yield
    end_time = datetime.now()
    activity_duration = end_time - start_time
    print("Activity duration(HH:MM:SS.ms):", activity_duration)
