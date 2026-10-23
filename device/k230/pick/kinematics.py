"""Portable FK and analytic IK for the existing equal-link arm and tilt choices."""
import math
from numeric import dot,norm,matmul,matvec,rotation,rigid,transform,identity


class Geometry:
    def __init__(self,model,pick,table_z_mm):
        self.model=model;self.pick=pick;self.table_z_m=table_z_mm/1000
        self.joints=model['joints'];self.points=model['points']
        self.open_angle=self.angle(6,pick['opening_count'])
    def angle(self,i,count):
        j=self.joints[i-1]
        return (count-j['zero_count'])*j['positive_direction']*2*math.pi/4096
    def angles(self,counts):return [self.angle(i,c) for i,c in enumerate(counts,1)]
    def counts(self,q):
        result=[]
        for j,a in zip(self.joints,q):
            c=j['zero_count']+j['positive_direction']*int(round(a*4096/(2*math.pi)))
            lo,hi=j['software_count_limits']
            if not lo<=c<=hi:raise ValueError('IK outside joint limits')
            result.append(c)
        return result
    def transforms(self,q):
        t=identity(4);result={}
        for j,a in zip(self.joints,q):
            t=matmul(matmul(t,j['origin_transform']),rigid(rotation(j['axis'],a),[0.,0.,0.]))
            result[j['child']]=t
        return result
    def tip_geometry(self,q):
        ts=self.transforms(q)
        pts={link:transform(ts[link],self.points[link]['tip']) for link in ('wan2','zhua')}
        return pts,ts
    def metrics(self,q):
        pts,ts=self.tip_geometry(q);lows={}
        for link in ('wan2','zhua'):
            t=ts[link]
            lows[link]=min(dot(t[2][:3],v)+t[2][3] for v in self.points[link]['hull'])
        return [(a+b)/2 for a,b in zip(pts['wan2'],pts['zhua'])],pts,lows,ts
    def alignment_midpoint(self,q):
        q=list(q);q[5]=self.open_angle;pts,t=self.tip_geometry(q)
        mid=[(a+b)/2 for a,b in zip(pts['wan2'],pts['zhua'])]
        direction=[pts['zhua'][i]-pts['wan2'][i] for i in range(2)];length=norm(direction)
        if length<1e-8:raise ValueError('Open jaw line has no horizontal projection')
        for i in range(2):mid[i]+=direction[i]/length*self.pick['cube_offset_toward_moving_finger_mm']/1000
        return mid
    def nominal_contact(self,q,orientation=None):
        t=self.transforms(q)['wan2'];axis=[t[0][1],t[1][1]];length=norm(axis)
        if length<1e-8:raise ValueError('Gripper across-axis has no horizontal projection')
        width=.020*(abs(axis[0])+abs(axis[1]))/length
        if orientation is not None:
            across=[t[i][1] for i in range(3)]
            width=.020*sum(abs(dot(across,v)) for v in orientation['axes_Base']+[orientation['table_normal_Base']])
        fixed=self.points['wan2']['distal'];moving=self.points['zhua']['distal']
        fixed_y=min(v[1] for v in fixed)
        def gap(a):
            c=math.cos(a);s=math.sin(a)
            return fixed_y-max(v[1]*c-v[2]*s-.0132 for v in moving)
        lo=0.;hi=math.pi/2;target=width-.0007
        if (gap(lo)-target)*(gap(hi)-target)>0:raise ValueError('No nominal contact opening')
        for unused in range(45):
            mid=(lo+hi)/2
            if gap(mid)<target:lo=mid
            else:hi=mid
        return (lo+hi)/2,width
    def solve(self,xy,height,tilt,seed,q5):
        theta=math.radians(180-tilt)
        # The first two pitch links end at Base z=.1075 and each have length .118.
        # Compute orientation-dependent tool offsets using the SAME exported FK.
        reference=[0.,0.,0.,theta,q5,self.open_angle]
        mid=self.alignment_midpoint(reference);low=min(self.metrics(reference)[2].values())
        offx=mid[0]+.013;offy=mid[1]
        target=[xy[0]+.013,xy[1]];rr=dot(target,target)-offy*offy
        if rr<0:raise ValueError('No yaw solution')
        z=self.table_z_m+height-(low-.236)
        pitch_z=z
        found=[]
        for sign in (1.,-1.):
            s=sign*math.sqrt(rr);yaw=math.atan2(target[1],target[0])-math.atan2(offy,s)
            yaw=(yaw+math.pi)%(2*math.pi)-math.pi
            pitch_x=s-offx
            cos3=(pitch_x*pitch_x+pitch_z*pitch_z-2*.118**2)/(2*.118**2)
            if not -1<=cos3<=1:continue
            for bend in (1.,-1.):
                a3=bend*math.acos(cos3)
                a2=math.atan2(pitch_x,pitch_z)-math.atan2(.118*math.sin(a3),.118+.118*math.cos(a3))
                q=[yaw,a2,a3,theta-a2-a3,q5,self.open_angle]
                try:counts=self.counts(q)
                except ValueError:continue
                qr=self.angles(counts);actual=self.alignment_midpoint(qr)
                error=norm([a-b for a,b in zip(actual[:2],xy)])
                herror=abs(min(self.metrics(qr)[2].values())-(self.table_z_m+height))
                if error<.002 and herror<.002:found.append((error,counts,qr))
        if not found:raise ValueError('No IK solution within joint limits for requested stage')
        found.sort(key=lambda v:(sum((a-b)**2 for a,b in zip(v[2][:4],seed[:4])),v[0]));return found[0][1],found[0][2]
    def report(self,counts):
        q=self.angles(counts);mid,pts,low,t=self.metrics(q)
        return {'alignment_reference':'open_fingertip_midpoint',
                'alignment_center_Base_mm':[v*1000 for v in self.alignment_midpoint(q)],
                'gripper_midpoint_Base_mm':[v*1000 for v in mid],
                'fixed_tip_center_Base_mm':[v*1000 for v in pts['wan2']],
                'moving_tip_center_Base_mm':[v*1000 for v in pts['zhua']],
                'link_clearance_above_table_mm':{k:(v-self.table_z_m)*1000 for k,v in low.items()},
                'cube_offset_toward_moving_finger_mm':self.pick['cube_offset_toward_moving_finger_mm']}
    def build_stages(self,xy,q0,tilt,q5,orientation=None):
        hover,qh=self.solve(xy,self.pick['hover_tip_height_mm']/1000,tilt,q0,q5)
        low,ql=self.solve(xy,self.pick['descend_tip_height_mm']/1000,tilt,qh,q5)
        angle,width=self.nominal_contact(ql,orientation)
        qc=list(ql);qc[5]=angle;nominal=self.counts(qc)[5]
        close=list(low);close[5]=nominal-self.pick['extra_closing_counts']
        for j,c in zip(self.joints,close):
            if not j['software_count_limits'][0]<=c<=j['software_count_limits'][1]:raise ValueError('Close target outside joint limits')
        lift,qu=self.solve(xy,self.pick['lift_tip_height_mm']/1000,tilt,ql,q5);lift[5]=close[5]
        return {'hover':hover,'descend':low,'close':close,'lift':lift,'tilt_deg':tilt,
                'extra_closing_counts':self.pick['extra_closing_counts'],
                'nominal_cube_width_mm':width*1000,
                'cube_offset_toward_moving_finger_mm':self.pick['cube_offset_toward_moving_finger_mm'],
                'target_xy_Base_mm':[v*1000 for v in xy],
                'target_FK':{k:self.report(v) for k,v in [('hover',hover),('descend',low),('close',close),('lift',lift)]}}
    def plan(self,xy,start,orientation=None):
        if orientation is not None and orientation.get('status')=='estimated':
            from wrist_alignment import plan_aligned
            try:return plan_aligned(self,xy,start,orientation)
            except ValueError as exc:
                if self.pick.get('unreachable_cube_orientation')!='keep_current_wrist':raise
                orientation={'status':'unavailable','reason':'Cube-aligned pose unavailable; user-selected original-wrist fallback',
                    'alignment_failure':str(exc)}
        q0=self.angles(start);q5=q0[4];failures=[]
        for tilt in sorted([20,25,30,35,40,45],key=lambda t:abs(t-25)):
            try:
                result=self.build_stages(xy,q0,tilt,q5)
                result['wrist_alignment']={'status':'held_current',
                    'reason':orientation.get('reason') if orientation else 'Orientation alignment not requested',
                    'servo5_goal':result['hover'][4]}
                if orientation and orientation.get('alignment_failure'):
                    result['wrist_alignment']['alignment_failure']=orientation['alignment_failure']
                return result
            except ValueError as exc:failures.append(str(exc))
        raise ValueError('No complete four-stage IK: '+str(failures))
