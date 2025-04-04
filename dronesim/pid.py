import numpy as np

class Controller():

    def __init__(self, Kp: float, Kd: float, Ki: float, Kisat: float, n: int):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.Kisat = Kisat
        
        # Integration total
        self.int = np.zeros(n, dtype=np.float64)
        self.previous_error = None

    def feedback(self, error: np.ndarray, dt: float, dedt: float | None=None, dbg=False) -> np.ndarray:
        self.int += error * dt
        if dedt is None:
            if self.previous_error is None:
                self.previous_error = error.copy()
            dedt = (error - self.previous_error) / dt
        self.previous_error = error.copy()

        ni = np.linalg.norm(self.int)
        if ni > self.Kisat:
            self.int = self.Kisat * self.int / ni
        if dbg:
            print(f'{self.Kp * error=}')
            print(f'{self.Ki * self.int=}')
            print(f'{self.Kd * dedt=}')
        des_acc = self.Kp * error + self.Ki * self.int + self.Kd * dedt
        return des_acc

