"""Six independently measured exterior edges of the current20mm cube.

An image-only convex-hull proposal covers views with two similarly sized
faces. The calibrated table-up/horizon supplies direction families, never a
pose/height objective for choosing pixels. Unsupported/narrow silhouettes
reject for the other same-frame localizers. No capture, actuator or file I/O.
"""
import math

DEFAULT_LAB_THRESHOLDS=((0,100,-10,127,-128,-35),)
OBJECT_POINTS=[[-10.,0.,20.],[-10.,20.,20.],[10.,20.,20.],
               [10.,20.,0.],[10.,0.,0.],[-10.,0.,0.]]
CORNER_LABELS=['A','E','F','G','C','D']
EDGE_NAMES=['top_depth','back_top','back_outer','small_bottom','front_bottom','front_outer']

def _signed_area(poly):
    return sum(a[0]*b[1]-a[1]*b[0]for a,b in zip(poly,poly[1:]+poly[:1]))/2


def _reduce_hull(poly):
    if len(poly)<6:raise ValueError('Fewer than6 observed outline branches')
    poly=[list(v)for v in poly]
    if _signed_area(poly)<0:poly.reverse()
    removed=[]
    while len(poly)>6:
        areas=[]
        for i,v in enumerate(poly):
            a,b=poly[i-1],poly[(i+1)%len(poly)]
            areas.append(abs((v[0]-a[0])*(b[1]-a[1])-(v[1]-a[1])*(b[0]-a[0]))/2)
        j=min(range(len(poly)),key=lambda i:areas[i])
        removed.append(dict(vertex=poly[j],triangle_area_px2=areas[j]));del poly[j]
    return poly,removed


def _label_hexagon(p,corners,poly,origin,intrinsics,extrinsics,plane):
    """Opposite branch pair ranked by observed direction versus table-up."""
    choices=[]
    for i in range(3):
        score=0.;deviations=[]
        for j in (i,i+3):
            a,b=poly[j],poly[(j+1)%6];dx,dy=b[0]-a[0],b[1]-a[1]
            length=math.sqrt(dx*dx+dy*dy)
            if length<25:raise ValueError('Exterior seed edge shorter than25px')
            pixel=[origin[0]+(a[0]+b[0])/2,origin[1]+(a[1]+b[1])/2]
            slope=corners._vertical_slope(p,pixel,intrinsics,extrinsics,plane)
            agreement=abs((dx*slope+dy)/(length*math.sqrt(1+slope*slope)))
            score+=agreement;deviations.append(math.degrees(math.acos(min(1.,agreement))))
        choices.append((score,i,deviations))
    score,i,deviations=max(choices)
    pair=[i,(i+3)%6]
    left=min(pair,key=lambda j:(poly[j][0]+poly[(j+1)%6][0])/2)
    # In clockwise image order, the left exterior vertical branch runs D->A.
    start=(left+1)%6
    result=poly[start:]+poly[:start]
    if result[0][1]>=result[5][1]or result[2][1]>=result[3][1]:
        raise ValueError('No consistent upper/lower vertical topology')
    return result,dict(vertical_pair_ranking=[dict(score=v[0],indices=[v[1],v[1]+3],deviation_deg=v[2])for v in choices],selected_pair=[i,i+3],selected_score=score)


def _parallel_bottom(p,im,hsv,a,b,top_edge,top_seed,origin,intrinsics,extrinsics,plane):
    """One image RGB line-bank on the corresponding horizontal direction family.

    Direction comes from the already observed parallel top edge intersecting
    the calibrated table horizon. The table OFFSET/height and any pose are
    unavailable here. Image RGB8 support/energy alone selects the intercept.
    """
    try:
        from ..pick.pose import undistort
    except ImportError:
        from pose import undistort
    K=[list(row)for row in intrinsics['camera_matrix']]
    K[0]=[v*1280/intrinsics['image_width']for v in K[0]]
    K[1]=[v*960/intrinsics['image_height']for v in K[1]]
    D=intrinsics['distortion_coefficients']
    T=extrinsics['T_Base_from_Camera_mm'];n=plane['normal']
    up=[sum(T[k][j]*n[k]for k in range(3))for j in range(3)]
    pixels=[p.intersect(top_edge[0],[1.,0.,-point[0]])for point in top_seed]
    ideal=[undistort([v[0]+origin[0],v[1]+origin[1]],K,D)for v in pixels]
    line=p.fit(ideal)
    vanish=[line[1]*up[2]-line[2]*up[1],line[2]*up[0]-line[0]*up[2],line[0]*up[1]-line[1]*up[0]]
    scale=math.sqrt(sum(v*v for v in vanish));vanish=[v/scale for v in vanish]
    center=[(a[k]+b[k])/2 for k in range(2)];ray=undistort([center[0]+origin[0],center[1]+origin[1]],K,D)+[1.]
    plus=p.project([ray[k]+vanish[k]*.0001 for k in range(3)],K,D)
    minus=p.project([ray[k]-vanish[k]*.0001 for k in range(3)],K,D)
    tangent=[plus[k]-minus[k]for k in range(2)];length=math.sqrt(sum(v*v for v in tangent));tangent=[v/length for v in tangent]
    if sum(tangent[k]*(b[k]-a[k])for k in range(2))<0:tangent=[-v for v in tangent]
    length=p.distance(a,b);normal=[-tangent[1],tangent[0]]
    aa=[center[k]-.5*length*tangent[k]for k in range(2)];bb=[center[k]+.5*length*tangent[k]for k in range(2)]
    along=p.np.array([[.15+.7*j/63 for j in range(64)]]);offsets=p.np.arange(-24,25).reshape((49,1))
    mx=aa[0]+(bb[0]-aa[0])*along+offsets*normal[0]
    my=aa[1]+(bb[1]-aa[1])*along+offsets*normal[1]
    strengths=p.np.array(p._native_contrasts(im,mx,my,2*normal[0],2*normal[1]))
    inside=p.cv2.remap(hsv,p.np.array(mx+5*normal[0],dtype=getattr(p.np,'float32',p.FLOAT)),p.np.array(my+5*normal[1],dtype=getattr(p.np,'float32',p.FLOAT)),p.cv2.INTER_NEAREST)
    blue=(inside[:,:,0]>=90)*(inside[:,:,0]<=140)*(inside[:,:,1]>=80)*(inside[:,:,2]>=35)
    support=p.np.sum((strengths>.08*.08)*blue,axis=1).tolist()
    choices=[i for i in range(49)if support[i]>=45]
    if not choices:raise ValueError('No originalRGB8-supported bottom line in parallel family')
    index=max(choices,key=lambda i:(support[i],float(p.np.sum(strengths[i,:]*blue[i,:])),-abs(i-24)))
    shift=index-24
    result=[[v[k]+shift*normal[k]for k in range(2)]for v in (aa,bb)]
    return result,dict(method='current_top_edge_table_horizon_direction_RGB_intercept',
        vanishing_direction_Camera=vanish,intercept_offset_px=shift,
        original_RGB8_proposal_support=int(support[index]),selected_seed=result,
        tested_intercepts=49,table_offset_used=False,pose_used=False)


def _context(p,bgr,bbox,prepared):
    if bgr.shape!=(960,1280,3):
        raise ValueError('Expected full1280x960 JPEG95-decoded BGR')
    if prepared is not None:
        return prepared
    ox,oy,x1,y1=p._roi(bbox,24,bgr.shape,1.6)
    raw=p.np.array(bgr[oy:y1,ox:x1],dtype=p.np.uint8)
    return {'raw':raw,'hsv':p.cv2.cvtColor(raw,p.cv2.COLOR_BGR2HSV),
        'smooth':p.cv2.GaussianBlur(raw,(3,3),.7),'bbox_px':list(bbox),
        'local_bbox':[bbox[0]-ox,bbox[1]-oy,bbox[2],bbox[3]],
        'roi_px':[ox,oy,x1-ox,y1-oy]}


def _proposal(p,corners,context,intrinsics,extrinsics,plane,thresholds):
    raw,hsv=context['raw'],context['hsv']
    x,y,w,h=context['local_bbox'];ox,oy=context['roi_px'][:2]
    lab=p.cv2.cvtColor(raw,p.cv2.COLOR_BGR2LAB)
    mask=p.np.zeros(hsv.shape[:2],dtype=p.np.uint8)
    for L0,L1,a0,a1,b0,b1 in thresholds:
        part=p.cv2.inRange(lab,(int(round(L0*255/100)),int(a0+128),int(b0+128)),
            (int(round(L1*255/100)),int(a1+128),int(b1+128)))
        mask=p.np.maximum(mask,part)
    contours,unused=p.cv2.findContours(mask,p.cv2.RETR_EXTERNAL,p.cv2.CHAIN_APPROX_SIMPLE)
    center=(int(round(x+.5*w)),int(round(y+.6*h)))
    containing=[c for c in contours if p.cv2.pointPolygonTest(c,center,False)>=0]
    body_kind='configured_LAB';hsv_thresholds=None
    if not containing:
        # Dark saturated-blue material can fall outside the LAB b proposal
        # while remaining inside the existing HSV body class. This is one
        # image-only retry, before any edge measurement or cube pose exists.
        values=hsv[max(0,int(y)):min(hsv.shape[0],int(y+h)),
                   max(0,int(x)):min(hsv.shape[1],int(x+w)),2]
        floor=max(35.,.25*float(p.np.max(values)))
        lower=(90,120,int(math.ceil(floor)));upper=(140,255,255)
        mask=p.cv2.inRange(hsv,lower,upper)
        contours,unused=p.cv2.findContours(mask,p.cv2.RETR_EXTERNAL,p.cv2.CHAIN_APPROX_SIMPLE)
        containing=[c for c in contours if p.cv2.pointPolygonTest(c,center,False)>=0]
        if not containing:raise ValueError('No current HSV body at coarse source center')
        body_kind='existing_HSV_body_when_LAB_center_absent'
        hsv_thresholds={'lower':list(lower),'upper':list(upper)}
    contour=max(containing,key=p.cv2.contourArea)
    hull=p.cv2.convexHull(contour)
    polygon=p.cv2.approxPolyDP(hull,.003*p.cv2.arcLength(hull,True),True).reshape((-1,2)).tolist()
    reduced,removed=_reduce_hull(polygon)
    seed,layout=_label_hexagon(p,corners,reduced,[ox,oy],intrinsics,extrinsics,plane)
    source=list(polygon)
    if _signed_area(source)<0:source.reverse()
    lines=[];arcs=[]
    for a,b in zip(seed,seed[1:]+seed[:1]):
        j=source.index(a);arc=[a]
        while arc[-1]!=b:
            j=(j+1)%len(source);arc.append(source[j])
        aa,bb=max(zip(arc,arc[1:]),key=lambda pair:p.distance(*pair))
        lines.append(p.fit([aa,bb]));arcs.append([list(aa),list(bb)])
    seed=[p.intersect(lines[i-1],lines[i])for i in range(6)]
    body=p.np.zeros(mask.shape,dtype=p.np.uint8)
    p.cv2.drawContours(body,[contour],-1,(255,),-1)
    ownership=p.np.array(hsv,dtype=p.np.uint8)
    ownership[:,:,2]=ownership[:,:,2]*(body>0)*(mask>0)
    return seed,contour,ownership,{'method':'current_body_hull_longest_arc_segments',
        'body_mask_kind':body_kind,'HSV_body_thresholds':hsv_thresholds,
        'mask_selection':'LAB_component_contains_source_center_else_existing_HSV_body',
        'LAB_thresholds':[list(v)for v in thresholds],
        'LAB_mapping':'OpenCV8_L_255over100_a_b_plus128',
        'LAB_threshold_source':'default_blue_approach'if thresholds is DEFAULT_LAB_THRESHOLDS else'explicit',
        'body_source_center_px':list(center),'hull_polygon_local_px':polygon,
        'longest_arc_segments_px':arcs,'table_direction_layout':layout,
        'pose_used':False,'table_offset_used':False}


def _measurement(p,edges,proposals,context,contour,audit):
    points=[p.intersect(edges[i-1][0],edges[i][0])for i in range(6)]
    # A rounded wall may give two well-fitted segments of the SAME physical
    # edge. It is not a sixth cube corner if their measured turn is unresolved
    # at the existing3px line precision. Check real lines, not hull seeds.
    if any(p.residual(points[i],p.fit([points[i-1],points[(i+1)%6]]))<=3
           for i in range(6)):
        raise ValueError('Cube outline has no distinct measured six-edge topology')
    seed=[p.intersect(proposals[i-1],proposals[i])for i in range(6)]
    halves=[[p.intersect(edges[i-1][2][j],edges[i][2][j])for i in range(6)]for j in (0,1)]
    errors=[p.distance(a,b)for group in halves for a,b in zip(points,group)]
    if max(errors)>3:raise ValueError('Cube outline interleaved corners disagree')
    if max(p.distance(a,b)for a,b in zip(points,seed))>18:
        raise ValueError('Cube outline corners too far from current image proposal')
    if p.area(points)<1000 or not p.cv2.isContourConvex(p.np.array(points,dtype=p.FLOAT)):
        raise ValueError('Cube outline is not convex/large')
    ox,oy=context['roi_px'][:2]
    full=[[v[0]+ox,v[1]+oy]for v in points]
    if any(not(3<a<1277 and 3<b<957)for a,b in full):raise ValueError('Cube outline clipped')
    support=p.cv2.approxPolyDP(contour,.003*p.cv2.arcLength(contour,True),True)
    if not p.component_support(points,{'component_contour_px':support.reshape((-1,2)).tolist()}):
        raise ValueError('Cube outline outside current blue component')
    def restore(line):
        a,b,c=line
        return[a,b,c-a*ox-b*oy]
    observed=[]
    for name,edge,source in zip(EDGE_NAMES,edges,audit):
        observed.append({'name':name,'quality':dict(edge[1]),
            'line_px':restore(edge[0]),'half_lines_px':[restore(v)for v in edge[2]],
            'image_source':source})
    return {'evidence_type':'nonplanar_cube_six_corners',
        'object_points_mm':[list(v)for v in OBJECT_POINTS],'image_points_px':full,
        'corner_labels':list(CORNER_LABELS),'observed_edges':observed,
        'half_corner_errors_px':errors,'roi_px':list(context['roi_px']),
        'topology':'silhouette_hexagon_left_image_face_front',
        'component_support':True,'foot_source':'two_measured_exterior_bottoms',
        'candidate_window_shared':True,'odd_even_check':'independent_measured_point_line_fits'}


def measure_steps(bgr,bbox_px,intrinsics,extrinsics,table_plane,prepared=None,lab_thresholds=None):
    """Yield image-only work and at most one six-corner measurement candidate.

    The current LAB body predicate owns exterior proposal windows. Only when
    it misses the source centre is the existing HSV body predicate used. Some
    tables reflect blue light: that is not broad-HSV outside-nonblue evidence.
    Such a window may use the original RGB8 contrast sampler, explicitly typed
    below, and still requires all64-station line/half/corner/component checks.
    The consumer independently applies the20mm cube3px/table pose policy.
    """
    try:
        from . import physical_side as p,edge_profiles as profiles,cube_corners as corners
    except ImportError:
        import physical_side as p
        import edge_profiles as profiles
        import cube_corners as corners
    count=0;rejected=[]
    try:
        if table_plane is None:raise ValueError('Cube outline requires the paired calibrated table')
        context=_context(p,bgr,bbox_px,prepared)
        thresholds=DEFAULT_LAB_THRESHOLDS if lab_thresholds is None else lab_thresholds
        seed,contour,ownership,proposal=_proposal(p,corners,context,intrinsics,extrinsics,table_plane,thresholds)
        yield {'stage':'cube_outline_proposal','roi_px':context['roi_px'],'proposal':proposal}
        im,hsv=context['smooth'],context['hsv'];origin=context['roi_px'][:2]
        lab_body=proposal['body_mask_kind']=='configured_LAB'
        edges=[];proposals=[];sources=[]
        for i,(a,b)in enumerate(zip(seed,seed[1:]+seed[:1])):
            source={'proposal_segment_local_px':[a,b],'pose_used':False}
            if i in (3,4):
                top=0 if i==3 else 1
                (a,b),family=_parallel_bottom(p,im,hsv,a,b,edges[top],
                    [seed[top],seed[(top+1)%6]],origin,intrinsics,extrinsics,table_plane)
                source['parallel_family']=family
            proposals.append(p.fit([a,b]))
            result=profiles.spread_edge(im,ownership,a,b,p._fit_edge,p._native_contrasts)
            if result is None:
                edge=None
                for event in p._edge_steps(im,a,b,hsv=hsv):
                    if event['stage']=='edge_complete':edge=event['edge']
                    else:yield {'stage':'cube_outline_samples','edge_name':EDGE_NAMES[i],'stations':event.get('stations')}
                if edge is None:raise ValueError('No original RGB8 cube outline edge')
                source['backend']='original_RGBcontrast_sampler_after_exterior_proposal'
                edge[1].update(appearance_method='original_RGB8_gradient_peak',
                    edge_ownership=('same_current_LAB_component_exterior_window'if lab_body
                                    else'same_current_HSV_body_exterior_window'),
                    broad_HSV_outside_nonblue_verified=False)
            else:
                edge=result[0];source['backend']='colour_adaptive_RGB_spread'
                edge[1].update(edge_ownership=('configured_LAB_body_exterior'if lab_body
                                             else'existing_HSV_body_exterior'),
                    broad_HSV_outside_nonblue_verified=False)
            source['body_mask_kind']=proposal['body_mask_kind']
            source['HSV_body_thresholds']=proposal['HSV_body_thresholds']
            source['LAB_mapping']=proposal['LAB_mapping']
            source['LAB_mapping_role']='ownership'if lab_body else'initial_proposal_probe_only'
            source['LAB_thresholds']=proposal['LAB_thresholds']
            edges.append(edge);sources.append(source)
            yield {'stage':'cube_outline_edge_complete','edge_name':EDGE_NAMES[i]}
        measurement=_measurement(p,edges,proposals,context,contour,sources)
        measurement['outline_proposal']=proposal
        count=1
        yield {'stage':'cube_candidate','candidate':0,'measurement':measurement}
    except (ValueError,ZeroDivisionError)as exc:
        rejected.append({'reason':str(exc)})
        yield {'stage':'cube_outline_rejected','reason':str(exc)}
    yield {'stage':'cube_outline_complete','candidate_count':count,'rejections':rejected}
