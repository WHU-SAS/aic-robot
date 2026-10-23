"""Find a feasible wrist roll that aligns projected jaw closure with a cube side."""
import math
from numeric import dot,cross,unit,rotation,matvec,transpose


def project(v,n):return [a-dot(v,n)*b for a,b in zip(v,n)]

def error(direction,target,n):
    d=unit(project(direction,n));t=unit(project(target,n))
    a=math.atan2(dot(n,cross(t,d)),dot(t,d))
    return (a+math.pi/2)%math.pi-math.pi/2


def closure(geometry,q):
    t=geometry.transforms(q)['wan2']
    return [t[i][1] for i in range(3)]


def roll_candidates(g,xy,tilt,orientation):
    theta=math.radians(180-tilt);n=orientation['table_normal_Base']
    bounds=sorted(g.angle(5,c) for c in g.joints[4]['software_count_limits'])
    # With joints 1..4 fixed, joint 5 rotates both tip points rigidly about
    # the wrist. Compute that small transform instead of repeating six-link FK
    # at every scalar root sample. Full FK verifies each final encoded target.
    reference=[0.,0.,0.,theta,0.,g.open_angle]
    tips,ts=g.tip_geometry(reference);t=ts['wan2'];R=[row[:3] for row in t[:3]];pivot=[t[i][3] for i in range(3)]
    local=[matvec(transpose(R),[v-o for v,o in zip(tips[k],pivot)]) for k in ('wan2','zhua')]
    offset=g.compensation_xy(reference)
    closing_local=g.closing_shift_local();closing_zero=matvec(R,closing_local)
    target=[xy[0]+.013,xy[1]]
    yaw_limits=sorted(g.angle(1,c) for c in g.joints[0]['software_count_limits'])
    cache={}
    def evaluate(roll,sign):
        if roll not in cache:
            r=rotation(g.joints[4]['axis'],roll)
            pts=[[o+v for o,v in zip(pivot,matvec(R,matvec(r,p)))] for p in local]
            closing_rotated=matvec(R,matvec(r,closing_local))
            offy=(pts[0][1]+pts[1][1])/2+offset[1]+closing_rotated[1]-closing_zero[1]
            direction=matvec(R,matvec(r,[0.,1.,0.]));rr=dot(target,target)-offy**2
            cache[roll]=(offy,direction,rr)
        offy,d,rr=cache[roll]
        if rr<=0:return None
        yaw=math.atan2(target[1],target[0])-math.atan2(offy,sign*math.sqrt(rr))
        yaw=(yaw+math.pi)%(2*math.pi)-math.pi
        if not yaw_limits[0]<=yaw<=yaw_limits[1]:return None
        c=math.cos(yaw);s=math.sin(yaw)
        return [c*d[0]-s*d[1],s*d[0]+c*d[1],d[2]]
    grid=[bounds[0]+(bounds[1]-bounds[0])*i/12 for i in range(13)]
    roots=[]
    for sign in (1.,-1.):
        for axis_index,axis in enumerate(orientation['axes_Base']):
            previous=None
            for roll in grid:
                d=evaluate(roll,sign)
                if d is None:previous=None;continue
                e=error(d,axis,n)
                if abs(e)<1e-5:roots.append((roll,axis_index,sign))
                if previous is not None:
                    lo,elo=previous
                    # Reject a sign flip across the +/-90 degree discontinuity.
                    if e*elo<0 and abs(e-elo)<math.pi/2:
                        hi=roll
                        for unused in range(12):
                            mid=(lo+hi)/2;direction=evaluate(mid,sign)
                            if direction is None:break
                            em=error(direction,axis,n)
                            if abs(em)<1e-5:lo=hi=mid;break
                            if em*elo<=0:hi=mid
                            else:lo=mid;elo=em
                        roots.append(((lo+hi)/2,axis_index,sign))
                previous=(roll,e)
    unique=[]
    for roll,index,sign in roots:
        if not any(abs(roll-r)<1e-4 and index==i and sign==s for r,i,s in unique):unique.append((roll,index,sign))
    return unique


def plan_aligned(g,xy,start,orientation):
    # The closing width depends on the selected pose. Re-solve using that
    # encoded closing goal until the alignment reference and command agree.
    reference=[0.,0.,0.,math.pi,0.,g.open_angle]
    contact,_=g.nominal_contact(reference)
    j=g.joints[5]
    g.alignment_close_count=j['zero_count']+j['positive_direction']*int(round(contact*4096/(2*math.pi)))-g.pick['extra_closing_counts']
    for iteration in range(4):
        plan=_search_aligned(g,xy,start,orientation)
        if plan['close'][5]==g.alignment_close_count:
            plan['wrist_alignment']['closing_reference_iterations']=iteration+1
            plan['wrist_alignment']['alignment_close_count']=g.alignment_close_count
            return plan
        g.alignment_close_count=plan['close'][5]
    raise ValueError('Closing midpoint reference did not converge to encoded goal')


def _search_aligned(g,xy,start,orientation):
    q0=g.angles(start);failures=[];outcomes={};height_excess={}
    attempts=0
    tilts=g.pick.get('wrist_alignment_tilts_deg',list(range(0,46,5)))
    def collect(values,branch=None):
        result=[]
        for tilt in values:
            for roll,index,sign in roll_candidates(g,xy,tilt,orientation):
                if branch is not None and (index,sign)!=branch:continue
                # Tip-height difference only depends on wrist orientation, not on
                # the shoulder/elbow translation or Base yaw. Rank before full IK.
                reference=[0.,0.,0.,math.radians(180-tilt),roll,g.open_angle]
                mid=g.alignment_midpoint(reference);target=[xy[0]+.013,xy[1]]
                reference[0]=math.atan2(target[1],target[0])-math.atan2(mid[1],sign*math.sqrt(dot(target,target)-mid[1]**2))
                _,_,low,_=g.metrics(reference)
                try:contact,width=g.nominal_contact(reference,orientation)
                except ValueError:continue
                closed=list(reference);closed[5]=contact-g.pick['extra_closing_counts']*2*math.pi/4096
                _,_,closed_low,_=g.metrics(closed)
                heights=[g.pick['descend_tip_height_mm']+(v-min(low.values()))*1000 for v in closed_low.values()]
                difference=abs(heights[0]-heights[1])
                # Prefer fingers able to overlap the 20mm cube at closure. These
                # scores rank geometric candidates, not new runtime motion gates.
                excess=max(0.,max(heights)-20.,-min(heights))
                score=(excess,int(difference/2.),abs(roll-q0[4]),difference,abs(tilt-25))
                result.append((score,tilt,roll,index,sign))
        return sorted(result,key=lambda item:item[0])
    def choose_high_tilt(height,low_tilt,roll,axis_index,seed):
        # Keep joint 5 fixed; prefer the nearest reachable pitch for each
        # elevated stage instead of forcing the grasp pitch at every height.
        choices=sorted(set(tilts+[low_tilt]),key=lambda t:(abs(t-low_tilt),t))
        for high_tilt in choices:
            try:
                _,q=g.solve(xy,height/1000,high_tilt,seed,roll)
                deviation=abs(error(closure(g,q),orientation['axes_Base'][axis_index],orientation['table_normal_Base']))
                if deviation<=math.pi/180:return high_tilt
            except ValueError:continue
        raise ValueError('No elevated IK with fixed wrist roll and side alignment')
    def attempt(items,search):
        nonlocal attempts
        for score,tilt,roll,index,sign in items:
            attempts+=1
            key=(round(tilt,6),index,sign)
            outcome='ik'
            try:
                # Reject unsuitable grasp poses before searching elevated
                # stages. The final encoded plan is checked again below.
                _,ql=g.solve(xy,g.pick['descend_tip_height_mm']/1000,tilt,q0,roll)
                contact,_=g.nominal_contact(ql,orientation)
                qc=list(ql);qc[5]=contact;closed=g.counts(qc)
                closed[5]-=g.pick['extra_closing_counts']
                preview=g.report(closed)['link_clearance_above_table_mm']
                outcome='height'
                height_excess[key]=max(0.,max(preview.values())-20.,-min(preview.values()))
                if height_excess[key]>0:
                    raise ValueError('Rotated fingers do not both overlap the 20mm cube height')
                outcome='collision'
                if g.cube_envelope is not None:
                    hit=g.cube_envelope.pose(g,ql)
                    if hit:raise ValueError('Open finger intersects cube at descend: '+str(hit))
                outcome='ik'
                high=choose_high_tilt(g.pick['hover_tip_height_mm'],tilt,roll,index,q0)
                lift=choose_high_tilt(g.pick['lift_tip_height_mm'],tilt,roll,index,ql)
                stage_tilts={'hover':high,'descend':tilt,'close':tilt,'lift':lift}
                plan=g.build_stages(xy,q0,tilt,roll,orientation,stage_tilts=stage_tilts)
                errors=[]
                for name in ('hover','descend','lift'):
                    q=g.angles(plan[name])
                    errors.append(abs(error(closure(g,q),orientation['axes_Base'][index],orientation['table_normal_Base']))*180/math.pi)
                outcome='direction'
                if max(errors)>1.:raise ValueError('Rounded wrist direction differs from selected side')
                outcome='height'
                clearances=plan['target_FK']['close']['link_clearance_above_table_mm']
                height_excess[key]=max(0.,max(clearances.values())-20.,-min(clearances.values()))
                if height_excess[key]>0:
                    raise ValueError('Rotated fingers do not both overlap the 20mm cube height')
                outcome='collision'
                g.validate_open_fingers(plan)
                q=g.angles(plan['descend']);_,tips,low,_=g.metrics(q)
                plan['wrist_alignment']={'status':'aligned','cube_yaw_table_deg':orientation['yaw_table_deg'],
                    'side_count':orientation['side_count'],'cube_axis_index':index,
                    'closure_direction_Base':closure(g,q),
                    'servo5_goal':plan['descend'][4],'servo5_angle_deg':q[4]*180/math.pi,
                    'servo5_change_deg':(q[4]-q0[4])*180/math.pi,
                    'stage_direction_errors_deg':errors,
                    'tilt_search':search,'candidate_attempts':attempts,
                    'stage_tilts_deg':dict(stage_tilts),
                    'finger_low_height_difference_mm':abs(low['wan2']-low['zhua'])*1000,
                    'tip_height_difference_mm':abs(tips['wan2'][2]-tips['zhua'][2])*1000,
                    'closed_finger_clearances_mm':plan['target_FK']['close']['link_clearance_above_table_mm']}
                return plan
            except ValueError as exc:
                outcomes[key]=outcome;failures.append(str(exc))
        return None
    result=attempt(collect(tilts),'coarse')
    if result is not None:return result
    # A 5-degree grid can skip the narrow overlap between arm reachability
    # and finger-height feasibility. Refine only intervals where those two
    # failure reasons meet; retain exactly the same encoded-goal checks.
    branches={};intervals=[]
    for (tilt,index,sign),reason in outcomes.items():
        branches.setdefault((index,sign),[]).append((tilt,reason))
    for branch,rows in branches.items():
        rows.sort()
        for (lo,a),(hi,b) in zip(rows,rows[1:]):
            if {a,b} in ({'ik','height'},{'ik','collision'}):
                key=(hi if b in ('height','collision') else lo,branch[0],branch[1])
                intervals.append((height_excess.get(key,0.),branch,lo,a,hi,b))
    # Try the closest height fit first. Bisection brackets the transition;
    # only the matching roll branch needs refinement, not every cube axis.
    for excess,branch,lo,a,hi,b in sorted(intervals):
        while hi-lo>1.:
            mid=round((lo+hi)/2,6)
            result=attempt(collect([mid],branch),'boundary_bisection')
            if result is not None:return result
            reason=outcomes.get((mid,branch[0],branch[1]))
            if reason==a:lo=mid
            elif reason==b:hi=mid
            else:break
        values=[i/10. for i in range(int(math.floor(lo*10))+1,int(math.ceil(hi*10)))
                if lo<i/10.<hi]
        result=attempt(collect(values,branch),'boundary_refined_0.1deg')
        if result is not None:return result
    raise ValueError('No feasible cube-aligned four-stage IK: '+str(sorted(set(failures))))
