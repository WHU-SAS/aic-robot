"""Bounded cube-yaw planning using the same FK and original-wrist fallback.

Only final encoded poses are checked here; this is not a swept-volume planner.
"""
import math
try:
    from .numeric import dot,cross,norm,unit,rotation,matvec,transpose
except ImportError:
    from numeric import dot,cross,norm,unit,rotation,matvec,transpose

TILTS=(0,5,7.5,10,15,20,25,30,35,40,45)
HIGH_TILTS=(0,5,10,15,20,25,30,35,40,45)
ROLL_INTERVALS=12
ROOT_STEPS=12
MAX_ROOTS_PER_TILT=4


def _close_steps(steps):
    close=getattr(steps,'close',None)
    if close is not None:close()


def _drain_steps(steps):
    try:
        for event in steps:
            if event['stage']=='complete':return event['result']
    finally:_close_steps(steps)
    raise RuntimeError('Wrist planning ended without a result')


def projected(v,n):return [a-dot(v,n)*b for a,b in zip(v,n)]


def direction_error(direction,axis,n):
    d=unit(projected(direction,n));a=unit(projected(axis,n))
    angle=math.atan2(dot(n,cross(a,d)),dot(a,d))
    return (angle+math.pi/2)%math.pi-math.pi/2


def jaw_direction(g,q):
    points=g.metrics(q)[1]
    return [points['zhua'][i]-points['wan2'][i] for i in range(3)]


def roll_candidates(g,xy,tilt,orientation,initial_roll,counts):
    return _drain_steps(roll_candidates_steps(g,xy,tilt,orientation,initial_roll,counts))


def roll_candidates_steps(g,xy,tilt,orientation,initial_roll,counts):
    theta=math.radians(180-tilt);n=orientation['table_normal_Base']
    ref=[0.,0.,0.,theta,0.,g.open_angle]
    _,pts,_,ts=g.metrics(ref);t=ts['wan2']
    R=[row[:3] for row in t[:3]];pivot=[t[i][3] for i in range(3)]
    inv=transpose(R)
    local=[matvec(inv,[p[i]-pivot[i] for i in range(3)]) for p in (pts['wan2'],pts['zhua'])]
    target=[xy[0]+.013,xy[1]]
    yaw_bounds=sorted(g.angle(1,c) for c in g.joints[0]['software_count_limits'])
    bounds=sorted(g.angle(5,c) for c in g.joints[4]['software_count_limits'])
    cache={}
    def evaluate(roll,sign):
        if roll not in cache:
            counts['scalar_evaluations']+=1
            rollR=rotation(g.joints[4]['axis'],roll)
            p=[matvec(R,matvec(rollR,v)) for v in local]
            offy=pivot[1]+(p[0][1]+p[1][1])/2
            d=[p[1][i]-p[0][i] for i in range(3)]
            # Use the same compensated open-midpoint as Geometry.solve.
            # Its fixed-to-moving direction rotates with the selected wrist.
            offset=g.pick['cube_offset_toward_moving_finger_mm']/1000
            if offset:
                length=norm(d[:2])
                if length<1e-8:return None
                offy+=offset*d[1]/length
            cache[roll]=(offy,d,dot(target,target)-offy*offy)
        offy,d,rr=cache[roll]
        if rr<=0:return None
        yaw=math.atan2(target[1],target[0])-math.atan2(offy,sign*math.sqrt(rr))
        yaw=(yaw+math.pi)%(2*math.pi)-math.pi
        if not yaw_bounds[0]<=yaw<=yaw_bounds[1]:return None
        c,s=math.cos(yaw),math.sin(yaw)
        return [c*d[0]-s*d[1],s*d[0]+c*d[1],d[2]]
    grid=[bounds[0]+(bounds[1]-bounds[0])*i/ROLL_INTERVALS for i in range(ROLL_INTERVALS+1)]
    roots=[]
    for sign in (1.,-1.):
        for index,axis in enumerate(orientation['axes_Base']):
            previous=None
            for roll in grid:
                d=evaluate(roll,sign)
                if d is None:previous=None;continue
                error=direction_error(d,axis,n)
                if abs(error)<1e-6:roots.append((roll,index,sign))
                if previous is not None:
                    lo,elo=previous
                    if error*elo<0 and abs(error-elo)<math.pi/2:
                        hi=roll
                        for unused in range(ROOT_STEPS):
                            mid=(lo+hi)/2;direction=evaluate(mid,sign)
                            if direction is None:break
                            em=direction_error(direction,axis,n)
                            if em*elo<=0:hi=mid
                            else:lo=mid;elo=em
                        roots.append(((lo+hi)/2,index,sign))
                previous=(roll,error)
            yield {'stage':'wrist_root_group','tilt_deg':tilt,'axis_index':index,'sign':sign}
    unique=[]
    for roll,index,sign in roots:
        if not any(abs(roll-v[0])<1e-4 and index==v[1] and sign==v[2] for v in unique):
            unique.append((roll,index,sign))
    unique.sort(key=lambda v:abs(v[0]-initial_roll))
    yield {'stage':'complete','result':unique[:MAX_ROOTS_PER_TILT]}
    return


def _elevated_stage(g,xy,height,low_tilt,seed,roll,counters):
    return _drain_steps(_elevated_stage_steps(g,xy,height,low_tilt,seed,roll,counters))


def _elevated_stage_steps(g,xy,height,low_tilt,seed,roll,counters):
    # Keep the original independent high-tilt ordering and solve count.
    for tilt in sorted(HIGH_TILTS,key=lambda t:(abs(t-low_tilt),t)):
        counters['ik_calls']+=1
        try:
            counts,q=g.solve(xy,height/1000,tilt,seed,roll)
        except ValueError:
            yield {'stage':'wrist_elevated_solve','height_mm':height,'tilt_deg':tilt,'accepted':False}
            continue
        yield {'stage':'wrist_elevated_solve','height_mm':height,'tilt_deg':tilt,'accepted':True}
        yield {'stage':'complete','result':(counts,q,tilt)}
        return
    raise ValueError('No elevated IK with the selected fixed wrist')


def _stages(g,xy,start,tilt,roll,orientation,counters):
    return _drain_steps(_stages_steps(g,xy,start,tilt,roll,orientation,counters))


def _stages_steps(g,xy,start,tilt,roll,orientation,counters):
    q0=g.angles(start)
    counters['ik_calls']+=1
    low,ql=g.solve(xy,g.pick['descend_tip_height_mm']/1000,tilt,q0,roll)
    yield {'stage':'wrist_low_solve','tilt_deg':tilt}
    # Both open jaws must already overlap the cube before closing. A high
    # moving jaw becoming low enough only after closure is not a valid entry.
    open_low=g.metrics(ql)[2]
    open_clearances={link:(value-g.table_z_m)*1000 for link,value in open_low.items()}
    if min(open_clearances.values())<0 or max(open_clearances.values())>20:
        raise ValueError('Open fingers do not both overlap the20mm cube height')
    angle,width=g.nominal_contact(ql,orientation)
    qc=list(ql);qc[5]=angle;nominal=g.counts(qc)[5]
    close=list(low);close[5]=nominal-g.pick['extra_closing_counts']
    for joint,value in zip(g.joints,close):
        if not joint['software_count_limits'][0]<=value<=joint['software_count_limits'][1]:
            raise ValueError('Close target outside joint limits')
    # Preserve the original table-Base-Z reference at target XY; these
    # clearances are not full-mesh distances to a tilted table plane.
    closed_low=g.metrics(g.angles(close))[2]
    clearances={link:(value-g.table_z_m)*1000 for link,value in closed_low.items()}
    if min(clearances.values())<0 or max(clearances.values())>20:
        raise ValueError('Closed fingers do not both overlap the20mm cube height')
    yield {'stage':'wrist_closing_geometry','tilt_deg':tilt}
    steps=_elevated_stage_steps(g,xy,g.pick['hover_tip_height_mm'],tilt,q0,roll,counters)
    try:
        for event in steps:
            if event['stage']=='complete':hover,qh,hover_tilt=event['result']
            else:
                detail=dict(event);detail['name']='hover';yield detail
    finally:_close_steps(steps)
    steps=_elevated_stage_steps(g,xy,g.pick['lift_tip_height_mm'],tilt,ql,roll,counters)
    try:
        for event in steps:
            if event['stage']=='complete':lift,qu,lift_tilt=event['result']
            else:
                detail=dict(event);detail['name']='lift';yield detail
    finally:_close_steps(steps)
    lift[5]=close[5]
    result={'hover':hover,'descend':low,'close':close,'lift':lift,'tilt_deg':tilt,
        'hover_tilt_deg':hover_tilt,'descend_tilt_deg':tilt,'lift_tilt_deg':lift_tilt,
        'stage_tilts_deg':{'hover':hover_tilt,'descend':tilt,'close':tilt,'lift':lift_tilt},
        'opening_count':g.pick['opening_count'],'nominal_closing_count':nominal,
        'extra_closing_counts':g.pick['extra_closing_counts'],'closing_count':close[5],
        'nominal_cube_width_mm':width*1000,
        'open_finger_clearances_mm':open_clearances,
        'open_finger_height_reference':'table_Base_Z_at_target_XY',
        'closed_finger_clearances_mm':clearances,
        'closed_finger_height_reference':'table_Base_Z_at_target_XY',
        'alignment_reference':'open_fingertip_midpoint',
        'cube_offset_toward_moving_finger_mm':g.pick['cube_offset_toward_moving_finger_mm'],
        'target_xy_Base_mm':[v*1000 for v in xy],
        'target_FK':{}}
    for name,goals in [('hover',hover),('descend',low),('close',close),('lift',lift)]:
        result['target_FK'][name]=g.report(goals)
        yield {'stage':'wrist_fk_report','name':name}
    yield {'stage':'complete','result':result}
    return


def _orientation(value):
    if not isinstance(value,dict) or value.get('status')!='estimated':return None
    try:
        vectors=value['axes_Base']+[value['table_normal_Base']]
        if len(vectors)!=3:return None
        for v in vectors:
            if (not isinstance(v,(list,tuple)) or len(v)!=3 or
                    any(type(x) not in (int,float) or not math.isfinite(x) for x in v) or
                    abs(norm(v)-1.)>1e-6):return None
        if any(abs(dot(vectors[i],vectors[j]))>1e-6 for i,j in ((0,1),(0,2),(1,2))):return None
        angle=value['yaw_table_deg']
        if type(angle) not in (int,float) or not math.isfinite(angle):return None
        return value
    except (KeyError,TypeError,ValueError):
        return None


def plan(g,xy,start,orientation,fallback):
    def fallback_steps(xy,start):
        yield {'stage':'complete','result':fallback(xy,start)}
    return _drain_steps(plan_steps(g,xy,start,orientation,fallback_steps))


def plan_steps(g,xy,start,orientation,fallback_steps):
    """Advance the unchanged wrist search; only the final event has a plan.

    Closing this generator closes its active child and skips all remaining
    candidates. No thresholds, iteration limits, or IK arithmetic are changed.
    """
    counters={'scalar_evaluations':0,'candidate_attempts':0,'ik_calls':0}
    reasons={};axis_preference=[];axis_min_rotation={}
    orientation=_orientation(orientation)
    q0=g.angles(start)
    if orientation is not None:
        # Rank the two equivalent face pairs across the bounded tilt grid.
        # Sorting only inside each tilt lets an early orthogonal solution win
        # before a slightly less upright, small-wrist-motion solution is tried.
        candidates=[]
        for tilt in TILTS:
            steps=roll_candidates_steps(g,xy,tilt,orientation,q0[4],counters)
            try:
                for event in steps:
                    if event['stage']=='complete':roots=event['result']
                    else:yield event
            finally:_close_steps(steps)
            candidates.append((tilt,roots))
            for roll,index,sign in roots:
                delta=abs(roll-q0[4])
                axis_min_rotation[index]=min(axis_min_rotation.get(index,delta),delta)
        axis_preference=sorted(axis_min_rotation,key=lambda i:(axis_min_rotation[i],i))
        for preferred_axis in axis_preference:
            for tilt,roots in candidates:
                for roll,index,sign in roots:
                    if index!=preferred_axis:continue
                    counters['candidate_attempts']+=1
                    yield {'stage':'wrist_candidate','tilt_deg':tilt,'axis_index':index,'roll_rad':roll,'sign':sign}
                    try:
                        steps=_stages_steps(g,xy,start,tilt,roll,orientation,counters)
                        try:
                            for event in steps:
                                if event['stage']=='complete':result=event['result']
                                else:yield event
                        finally:_close_steps(steps)
                        stage_errors={}
                        for stage in ('hover','descend','lift'):
                            stage_errors[stage]=math.degrees(abs(direction_error(jaw_direction(g,g.angles(result[stage])),
                                orientation['axes_Base'][index],orientation['table_normal_Base'])))
                            yield {'stage':'wrist_direction_report','name':stage}
                        # Heading matters at the grasp pose. Elevated-stage
                        # headings are recorded rather than rejecting good reach.
                        if stage_errors['descend']>1.:
                            raise ValueError('Rounded descend target does not match the selected side direction')
                        goal_angle=g.angles(result['descend'])[4]
                        result['wrist_alignment']={'status':'aligned','cube_yaw_table_deg':orientation['yaw_table_deg'],
                            'cube_axis_index':index,'servo5_goal':result['descend'][4],
                            'servo5_angle_deg':math.degrees(goal_angle),
                            'servo5_delta_from_start_deg':math.degrees(goal_angle-q0[4]),
                            'servo5_rotation_from_start_deg':math.degrees(abs(goal_angle-q0[4])),
                            'selection_rule':'nearest_face_pair_then_upright_tilt',
                            'axis_preference':list(axis_preference),
                            'axis_min_rotation_deg':{str(i):math.degrees(v) for i,v in axis_min_rotation.items()},
                            'direction_error_deg':[stage_errors['hover'],stage_errors['descend']],
                            'stage_direction_errors_deg':stage_errors,
                            'stage_tilts_deg':dict(result['stage_tilts_deg']),'counts':dict(counters),
                            'tilt_candidates_deg':list(TILTS),'failures':reasons}
                        yield {'stage':'complete','result':result}
                        return
                    except ValueError as exc:
                        reason=str(exc);reasons[reason]=reasons.get(reason,0)+1
                        yield {'stage':'wrist_candidate_rejected','tilt_deg':tilt,'axis_index':index,'reason':reason}
    else:reasons['No verified side orientation']=1
    yield {'stage':'wrist_fallback'}
    steps=fallback_steps(xy,start)
    try:
        for event in steps:
            if event['stage']=='complete':result=event['result']
            else:yield event
    finally:_close_steps(steps)
    result['wrist_alignment']={'status':'held_current_fallback','reason':'No complete cube-aligned four-stage pose',
        'servo5_goal':result['hover'][4],
        'servo5_delta_from_start_deg':math.degrees(g.angles(result['hover'])[4]-q0[4]),
        'servo5_rotation_from_start_deg':math.degrees(abs(g.angles(result['hover'])[4]-q0[4])),
        'selection_rule':'nearest_face_pair_then_upright_tilt',
        'axis_preference':list(axis_preference),
        'axis_min_rotation_deg':{str(i):math.degrees(v) for i,v in axis_min_rotation.items()},
        'counts':counters,'failures':reasons}
    yield {'stage':'complete','result':result}
    return
