from threading import Thread
from queue import Queue


class PrintWorker:
    def __init__(self):
        self.thread = Thread(target=self._loop, daemon=True)
        self.task_queue = Queue()
        self._killed = False
        self.start = self.thread.start
    
    def _loop(self):
        while 1:
            args, kwargs = self.task_queue.get()
            if args is None or kwargs is None:
                break
            print(*args, **kwargs)

    def print(self, *args, **kwargs):
        self.task_queue.put((args, kwargs))
    
    def kill(self):
        if not self._killed:
            self._killed = True
            self.task_queue.put((None, None))
    
    def __del__(self):
        self.kill()
