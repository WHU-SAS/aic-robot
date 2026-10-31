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


def projected(v,n):return [a-dot(v,n)*b for a,b in zip(v,n)]


def direction_error(direction,axis,n):
    d=unit(projected(direction,n));a=unit(projected(axis,n))
    angle=math.atan2(dot(n,cross(a,d)),dot(a,d))
    return (angle+math.pi/2)%math.pi-math.pi/2


def jaw_direction(g,q):
    points=g.metrics(q)[1]
    return [points['zhua'][i]-points['wan2'][i] for i in range(3)]


def roll_candidates(g,xy,tilt,orientation,initial_roll,counts):
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
    unique=[]
    for roll,index,sign in roots:
        if not any(abs(roll-v[0])<1e-4 and index==v[1] and sign==v[2] for v in unique):
            unique.append((roll,index,sign))
    unique.sort(key=lambda v:abs(v[0]-initial_roll))
    return unique[:MAX_ROOTS_PER_TILT]


def _elevated_stage(g,xy,height,low_tilt,seed,roll,counters):
    # Elevated stages retain the same wrist count and midpoint, but need not
    # match a side heading while both jaws are well above the cube.
    for tilt in sorted(HIGH_TILTS,key=lambda t:(abs(t-low_tilt),t)):
        counters['ik_calls']+=1
        try:
            counts,q=g.solve(xy,height/1000,tilt,seed,roll)
            return counts,q,tilt
        except ValueError:
            pass
    raise ValueError('No elevated IK with the selected fixed wrist')


def _stages(g,xy,start,tilt,roll,orientation,counters):
    q0=g.angles(start)
    counters['ik_calls']+=1
    low,ql=g.solve(xy,g.pick['descend_tip_height_mm']/1000,tilt,q0,roll)
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
    hover,qh,hover_tilt=_elevated_stage(g,xy,g.pick['hover_tip_height_mm'],tilt,q0,roll,counters)
    lift,qu,lift_tilt=_elevated_stage(g,xy,g.pick['lift_tip_height_mm'],tilt,ql,roll,counters)
    lift[5]=close[5]
    return {'hover':hover,'descend':low,'close':close,'lift':lift,'tilt_deg':tilt,
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
        'cube_offset_toward_moving_finger_mm':0,
        'target_xy_Base_mm':[v*1000 for v in xy],
        'target_FK':{k:g.report(v) for k,v in [('hover',hover),('descend',low),('close',close),('lift',lift)]}}


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
    if g.pick['cube_offset_toward_moving_finger_mm']!=0:
        raise ValueError('Wrist alignment requires zero empirical offset')
    counters={'scalar_evaluations':0,'candidate_attempts':0,'ik_calls':0}
    reasons={}
    orientation=_orientation(orientation)
    if orientation is not None:
        q0=g.angles(start)
        for tilt in TILTS:
            for roll,index,sign in roll_candidates(g,xy,tilt,orientation,q0[4],counters):
                counters['candidate_attempts']+=1
                try:
                    result=_stages(g,xy,start,tilt,roll,orientation,counters)
                    stage_errors={stage:math.degrees(abs(direction_error(jaw_direction(g,g.angles(result[stage])),
                        orientation['axes_Base'][index],orientation['table_normal_Base'])))
                        for stage in ('hover','descend','lift')}
                    # Heading matters at the grasp pose. Elevated-stage
                    # headings are recorded rather than rejecting good reach.
                    if stage_errors['descend']>1.:
                        raise ValueError('Rounded descend target does not match the selected side direction')
                    result['wrist_alignment']={'status':'aligned','cube_yaw_table_deg':orientation['yaw_table_deg'],
                        'cube_axis_index':index,'servo5_goal':result['descend'][4],
                        'servo5_angle_deg':math.degrees(g.angles(result['descend'])[4]),
                        'direction_error_deg':[stage_errors['hover'],stage_errors['descend']],
                        'stage_direction_errors_deg':stage_errors,
                        'stage_tilts_deg':dict(result['stage_tilts_deg']),'counts':dict(counters),
                        'tilt_candidates_deg':list(TILTS),'failures':reasons}
                    return result
                except ValueError as exc:
                    reason=str(exc);reasons[reason]=reasons.get(reason,0)+1
    else:reasons['No verified side orientation']=1
    result=fallback(xy,start)
    result['wrist_alignment']={'status':'held_current_fallback','reason':'No complete cube-aligned four-stage pose',
        'servo5_goal':result['hover'][4],'counts':counters,'failures':reasons}
    return result
