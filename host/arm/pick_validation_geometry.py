"""Formal PC trial: center the open fingertips before descending around a cube."""
import math
import numpy as np
from host.arm.blue_pick_geometry import PickGeometry


class OpenTipPickGeometry(PickGeometry):
    reference = 'open_fingertip_midpoint'

    def __init__(self,*args,cube_offset_toward_moving_finger_mm=0,**kwargs):
        super().__init__(*args,**kwargs)
        self.cube_offset_mm=float(cube_offset_toward_moving_finger_mm)
        if not math.isfinite(self.cube_offset_mm):
            raise ValueError('Cube offset must be finite')

    def open_reference(self,q):
        reference_q=np.array(q,dtype=float,copy=True)
        reference_q[5]=self.opening_angle_rad
        return self.metrics(reference_q)

    def alignment_midpoint(self,q):
        # Keep the alignment reference at the selected opening even when
        # reporting closed-jaw feedback. Closing targets still use nominal_contact.
        midpoint,points,unused_low,unused_transform=self.open_reference(q)
        # Positive: cube lies toward moving jaw within the opening. Negative:
        # toward fixed jaw. Offset rotates with the actual open-tip line in Base XY.
        direction=points['zhua'][:2]-points['wan2'][:2]
        length=float(np.linalg.norm(direction))
        if length<1e-8:raise ValueError('Open jaw line has no horizontal projection')
        midpoint=midpoint.copy()
        midpoint[:2]+=direction/length*self.cube_offset_mm/1000
        return midpoint

    def report(self,counts):
        result=super().report(counts)
        result.update(alignment_reference=self.reference,
                      alignment_opening_count=self.opening_count,
                      cube_offset_toward_moving_finger_mm=self.cube_offset_mm,
                      open_tip_midpoint_Base_mm=(self.open_reference(self.angles(counts))[0]*1000).tolist())
        return result

    def plan(self,xy,start):
        result=super().plan(xy,start)
        result.update(alignment_reference=self.reference,
                      alignment_opening_count=self.opening_count,
                      cube_offset_toward_moving_finger_mm=self.cube_offset_mm)
        return result
