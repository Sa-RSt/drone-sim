import math

class Clock:
    def __init__(self):
        self.duration = 0.
    
    def tick(self, dt, actual_time_difference):
        self.duration += dt
        return 1/actual_time_difference
    
    def __repr__(self):
        integer = int(math.floor(self.duration))
        fractional = self.duration - integer
        minutes, seconds = divmod(integer, 60)
        return ':'.join((str(minutes).zfill(2), str(seconds).zfill(2), f'{fractional:.6f}'[1:]))

    def reset(self):
        self.duration = 0.
