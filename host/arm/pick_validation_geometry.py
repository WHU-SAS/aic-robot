"""Formal PC trial: center the open fingertips before descending around a cube."""
import numpy as np
from host.arm.blue_pick_geometry import PickGeometry


class OpenTipPickGeometry(PickGeometry):
    reference = 'open_fingertip_midpoint'

    def alignment_midpoint(self,q):
        # Keep the alignment reference at the selected opening even when
        # reporting closed-jaw feedback. Closing targets still use nominal_contact.
        reference_q=np.array(q,dtype=float,copy=True)
        reference_q[5]=self.opening_angle_rad
        return self.metrics(reference_q)[0]

    def report(self,counts):
        result=super().report(counts)
        result.update(alignment_reference=self.reference,
                      alignment_opening_count=self.opening_count)
        return result

    def plan(self,xy,start):
        result=super().plan(xy,start)
        result.update(alignment_reference=self.reference,
                      alignment_opening_count=self.opening_count)
        return result
