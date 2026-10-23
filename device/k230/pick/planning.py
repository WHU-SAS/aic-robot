"""A user-selected deadline for planning only; no motion is dispatched here."""
import time


class PlanningTimeout(RuntimeError):
    def __init__(self,limit_ms,elapsed_ms):
        self.details={'limit_ms':limit_ms,'elapsed_ms':elapsed_ms,'motion_dispatched':False}
        super().__init__('Planning search timed out after '+str(elapsed_ms)+'ms (limit '+str(limit_ms)+'ms)')


def milliseconds():
    if hasattr(time,'ticks_ms'):return time.ticks_ms()
    return int(time.monotonic()*1000)


class PlanningBudget:
    def __init__(self,limit_ms,clock=None,difference=None):
        if isinstance(limit_ms,bool) or not isinstance(limit_ms,(int,float)) or not 0<limit_ms<1000000:
            raise ValueError('Invalid planning deadline')
        self.limit_ms=int(limit_ms);self.clock=clock or milliseconds
        self.difference=difference or (time.ticks_diff if hasattr(time,'ticks_diff') else lambda a,b:a-b)
        self.started=self.clock()
    def check(self):
        elapsed=self.difference(self.clock(),self.started)
        if elapsed>=self.limit_ms:raise PlanningTimeout(self.limit_ms,elapsed)
