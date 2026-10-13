"""Encoder-only continuous position stability; no hardware access."""
class PositionSettled:
 def __init__(self,seconds=0.5,tolerance=1):
  self.seconds=seconds;self.tolerance=tolerance;self.since=None;self.low=None;self.high=None
 def update(self,position,timestamp):
  position=int(position)
  if self.since is None:
   self.since=timestamp;self.low=self.high=position
  else:
   low=min(self.low,position);high=max(self.high,position)
   if high-low>2*self.tolerance:
    self.since=timestamp;self.low=self.high=position
   else:self.low=low;self.high=high
  return timestamp-self.since>=self.seconds
 def report(self,timestamp):
  return {'duration_s':timestamp-self.since,'position_min':self.low,'position_max':self.high,'tolerance_counts':self.tolerance,'required_duration_s':self.seconds}
