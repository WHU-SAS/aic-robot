"""Reusable blue-20mm-cube feature detection. No hardware initialization or transport."""
from blue_face_core import extract, component_boxes, native_rect_faces

def same_face_key(face):
    return (face.get('detector','fitted_edges'),face.get('kind'),face.get('face_index'),face.get('value_floor',40))

def outlines_match(a,b):
    for source in [a,a[::-1]]:
        for offset in range(6):
            rotated=source[offset:]+source[:offset]
            if max((x[0]-y[0])**2+(x[1]-y[1])**2 for x,y in zip(rotated,b))<=25:return True
    return False

def matched_three_frames(frames):
    """Bounded burst with three agreeing physical faces or cube outlines."""
    current_outlines=frames[-1].get('cube_outlines',[])
    has_outlines=any(frame.get('cube_outlines') for frame in frames)
    if current_outlines:
        for current in current_outlines:
            count=1
            for previous in frames[:-1]:
                if any(outlines_match(old['corners_px'],current['corners_px']) for old in previous.get('cube_outlines',[])):count+=1
                if count>=3:return True
    for current in frames[-1].get('candidates',[]):
        # Keep fast exit for a reliable modern face; legacy mixed-plane
        # fits wait for three whole-cube outlines when outlines are present.
        if current.get('detector','fitted_edges')=='fitted_edges' and (has_outlines or current.get('kind')=='side'):continue
        count=1;key=same_face_key(current)
        for previous in frames[:-1]:
            for face in previous.get('candidates',[]):
                if same_face_key(face)!=key:continue
                if max((a[0]-b[0])**2+(a[1]-b[1])**2 for a,b in zip(face['quad_px'],current['quad_px']))<=25:
                    count+=1;break
            if count>=3:return True
    return False


def validate_config(config):
    if config.get('schema') != 'aic.blue_cube_config.v1':
        raise ValueError('Unsupported blue cube configuration')
    camera = config['camera']
    if (camera['width'], camera['height']) != (1280, 960):
        raise ValueError('Only the validated 1280x960 profile is supported')
    if camera['hmirror'] or camera['vflip'] or config['object'] != {'color': 'blue', 'edge_mm': 20}:
        raise ValueError('Only the validated unmirrored blue20mm profile is supported')
    x, y, width, height = config['detector']['roi']
    if min(x, y) < 0 or min(width, height) <= 0 or x+width > 1280 or y+height > 960:
        raise ValueError('ROI outside image')
    if not 3 <= config['sampling']['frame_limit'] <= 9:
        raise ValueError('Frame limit must remain within the validated 3..9 range')
    return config


class BlueCubeDetector:
    def __init__(self, config):
        self.config = validate_config(config)
        self.width = config['camera']['width']
        self.height = config['camera']['height']
        self.detector = config['detector']
        self.thresholds = [tuple(t) for t in self.detector['lab_thresholds']]

    def detect_frame(self, img):
        blobs = img.find_blobs(self.thresholds, roi=tuple(self.detector['roi']),
                               pixels_threshold=self.detector['pixels_threshold'],
                               area_threshold=self.detector['area_threshold'], merge=False)
        blobs = sorted(blobs, key=lambda blob: blob.pixels(), reverse=True)
        if not blobs:
            return {'status':'no_blue_cube','candidates':[]}
        # Refine broad LAB bounds in HSV first, including LAB blobs touching
        # the image border. A clipped background is not a clipped cube.
        components=[];blob_records=[]
        for blob in blobs[:self.detector['max_blobs']]:
            boxes=component_boxes(img.get_pixel,blob.rect(),self.width,self.height)
            blob_records.append({'bbox_px':list(blob.rect()),'pixels':blob.pixels(),'HSV_components':boxes})
            for box in boxes:
                if box['clipped'] and box.get('clipped_sides')!=['bottom']:continue
                # De-duplicate HSV components from overlapping LAB proposals.
                if any(abs(box['bbox_px'][0]-c['bbox_px'][0])<8 and abs(box['bbox_px'][1]-c['bbox_px'][1])<8 and abs(box['bbox_px'][2]-c['bbox_px'][2])<12 and abs(box['bbox_px'][3]-c['bbox_px'][3])<12 for c in components):continue
                components.append(box)
        components.sort(key=lambda c:c['sampled_pixels'],reverse=True)
        attempts=[];result=None
        for box in components[:self.detector['max_components']]:
            candidate=extract(img.get_pixel,box['bbox_px'],self.width,self.height)
            native,native_info=native_rect_faces(img,box['bbox_px'],self.width,self.height)
            # A seam-free whole blue silhouette is not a validated square face.
            # Preserve known top/side fits; replace the unsafe unknown fallback.
            candidate['candidates']=[f for f in candidate.get('candidates',[]) if f['kind']!='unknown' or f.get('detector')=='cube_edges']+native
            candidate['native_rect_diagnostic']=native_info
            candidate['status']='faces_extracted' if candidate['candidates'] else ('cube_outline_extracted' if candidate.get('cube_outlines') else 'no_reliable_face')
            attempts.append({'status':candidate.get('status'),'bbox_px':candidate.get('bbox_px'),'rejected':candidate.get('rejected',[]),'candidate_count':len(candidate.get('candidates',[])),'cube_outlines':candidate.get('cube_outlines',[]),'native_rect_diagnostic':native_info,'three_face_diagnostic':candidate.get('three_face_diagnostic')})
            if candidate.get('candidates') or candidate.get('cube_outlines'):
                result=candidate;break
        if result is None:
            result={'status':'no_complete_HSV_face','candidates':[],'reason':'No complete physical blue face after refining LAB blobs'}
        result['LAB_blobs']=blob_records
        result['component_attempts']=attempts
        return result
