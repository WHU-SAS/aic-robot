"""Runtime face geometry extracted unchanged from grasp analysis."""
import cv2
import numpy as np

def ordered_quad(q):
    q = np.asarray(q, dtype=float).reshape(-1,2)
    angle = np.arctan2(q[:,1]-q[:,1].mean(), q[:,0]-q[:,0].mean())
    q = q[np.argsort(angle)]
    return np.roll(q, -np.argmin(q.sum(axis=1)), axis=0)

def face_candidates(im, group):
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    height, width = im.shape[:2]
    variants = []
    masks = []
    for value in [160,180,200,220]:
        mask = cv2.inRange(hsv, np.array([90,160,value]), np.array([140,255,255]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3,3),np.uint8))
        masks.append(mask)
        contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: continue
        co = max(contours, key=cv2.contourArea)
        if cv2.contourArea(co) < 1000: continue
        hull = cv2.convexHull(co)
        q = cv2.approxPolyDP(hull, .02*cv2.arcLength(hull,True), True)
        x,y,w,h = cv2.boundingRect(co)
        clipped = x <= 2 or y <= 2 or x+w >= width-2 or y+h >= height-2
        variants.append({'threshold_V':value, 'area':float(cv2.contourArea(co)),
                         'bbox':[x,y,w,h], 'polygon_px':q.reshape(-1,2).tolist(),
                         'clipped':bool(clipped), 'is_quad':len(q)==4})
    accepted = [v for v in variants if v['is_quad'] and not v['clipped']]
    result = {'group':group, 'variants':variants,
              'status':'complete_face_candidate' if accepted else 'no_complete_face',
              'face_identity':'automatic candidate; only group 1 operator-confirmed'}
    if accepted:
        quads = np.array([ordered_quad(v['polygon_px']) for v in accepted])
        q = np.median(quads, axis=0)
        span = np.ptp(quads, axis=0)
        if np.max(span) > 12:
            result['status'] = 'unstable_face_boundary'
        result.update(face_quad_px=q.tolist(), corner_span_px=span.tolist(),
                      thresholds_used=[v['threshold_V'] for v in accepted])
    elif variants:
        result['reason'] = 'Blue object reaches image boundary' if any(v['clipped'] for v in variants) else 'No stable four-corner face'
    else:
        result['reason'] = 'No sufficiently large bright saturated blue face detected'
    return result

def pose_candidates(q, K, D):
    obj = np.array([[0,0,0],[20,0,0],[20,20,0],[0,20,0]],float)
    solutions = cv2.solvePnPGeneric(obj, np.asarray(q,float), K,D, flags=cv2.SOLVEPNP_IPPE)
    result = []
    for rv,tv in zip(solutions[1],solutions[2]):
        R,_ = cv2.Rodrigues(rv)
        t = tv.ravel()
        if np.any((obj@R.T+t)[:,2] <= 0): continue
        predicted,_ = cv2.projectPoints(obj,rv,tv,K,D)
        error = float(np.sqrt(np.mean(np.sum((predicted.reshape(-1,2)-q)**2,axis=1))))
        face_center = R@np.array([10,10,0])+t
        inward = R[:,2].copy()
        if inward@face_center < 0: inward *= -1
        center = face_center+10*inward
        pixel,_ = cv2.projectPoints(center.reshape(1,3),np.zeros(3),np.zeros(3),K,D)
        undist = cv2.undistortPoints(pixel,K,D).reshape(2)
        result.append({'face_reprojection_rms_px':error,
                       'cube_center_Camera_mm':center.tolist(),
                       'cube_center_pixel':pixel.reshape(2).tolist(),
                       'cube_center_normalized_ray':undist.tolist(),
                       'inward_face_normal_Camera':inward.tolist()})
    return sorted(result,key=lambda a:a['face_reprojection_rms_px'])
