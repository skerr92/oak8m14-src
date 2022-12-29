from cocotb import coroutine
from cocotb.triggers import RisingEdge
from cocotb_bus.monitors import Monitor


class RisingEdgeCounter(Monitor):
    """Counts the number of rising edges for the given signal"""

    def __init__(self, name, signal, callback=None, event=None):
        self.name = name
        self.signal = signal
        self.counter = 0
        Monitor.__init__(self, callback, event)

    def reset(self):
        self.counter = 0

    @coroutine
    def _monitor_recv(self):
        clkedge = RisingEdge(self.signal)

        while True:
            # Capture signal at rising edge of clock
            yield clkedge
            self.counter += 1
            self._recv(self.counter)