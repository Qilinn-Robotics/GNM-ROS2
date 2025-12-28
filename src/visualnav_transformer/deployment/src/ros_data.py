import time

class ROSData:
    def __init__(self, timeout: int = 3, queue_size: int = 1, name: str = "", clock=None):
        self.timeout = timeout
        self.last_time_received = float("-inf")
        self.queue_size = queue_size
        self.data = None
        self.name = name
        self.phantom = False
        self.clock = clock

    def get_time(self):
        if self.clock is None:
            return time.time()
        return self.clock.now().nanoseconds / 1e9

    def get(self):
        return self.data

    def set(self, data):
        self.last_time_received = self.get_time()
        if self.queue_size == 1:
            self.data = data
        else:
            if self.data is None: 
                self.data = []
            
            # Note: Checking timeout here for clearing queue might be tricky if set is called frequently
            # simpler to just append and manage size. 
            
            if len(self.data) == self.queue_size:
                self.data.pop(0)
            self.data.append(data)

    def is_valid(self, verbose: bool = False):
        time_waited = self.get_time() - self.last_time_received
        valid = time_waited < self.timeout
        if self.queue_size > 1:
            valid = valid and (self.data is not None) and (len(self.data) == self.queue_size)
        
        if verbose and not valid:
            print(
                f"Not receiving {self.name} data for {time_waited:.2f} seconds (timeout: {self.timeout} seconds)"
            )
        return valid
