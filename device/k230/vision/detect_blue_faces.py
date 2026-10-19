"""CanMV K230 one-shot feature extraction. Contains no servo commands.
The PC launcher bundles blue_face_core.py automatically; IDE use needs that
file alongside this script on the SD card. Set PREVIEW=True for IDE display.
"""
import os, time, sys
import ujson as json
import uhashlib as hashlib
import binascii
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager
from blue_face_core import extract, median, component_boxes, native_rect_faces

PREVIEW = False
DEBUG_IMAGE_REMOTE = None
LOCAL_ROI_REMOTE = None
DEBUG_IMAGE_ROI = False
WIDTH, HEIGHT = 1280, 960
MAX_FRAMES = 9
BLUE_LAB = [(0, 100, -20, 127, -128, -8)]
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

sensor = None
media_ready = False
display_ready = False
started = time.ticks_ms()
debug_saved = False
sensor_stopped = False
error_occurred = False
try:
    print('CAPTURE_STAGE initialize')
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(width=WIDTH, height=HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)
    sensor.set_hmirror(False)
    sensor.set_vflip(False)
    Display.init(Display.VIRT, width=WIDTH, height=HEIGHT, fps=30, to_ide=PREVIEW)
    display_ready = True
    MediaManager.init()
    media_ready = True
    sensor.run()
    for _ in range(30):
        os.exitpoint()
        img = sensor.snapshot()
        time.sleep_ms(30)
    initialized = time.ticks_ms()
    print('CAPTURE_STAGE extract_faces')
    frames = []
    for index in range(MAX_FRAMES):
        os.exitpoint()
        img = sensor.snapshot()
        blobs = img.find_blobs(BLUE_LAB, roi=(0,432,WIDTH,HEIGHT-432),
                               pixels_threshold=1000, area_threshold=6000, merge=False)
        blobs = sorted(blobs, key=lambda blob: blob.pixels(), reverse=True)
        if not blobs:
            frames.append({'status':'no_blue_cube','candidates':[]})
            continue
        # Refine broad LAB bounds in HSV first, including LAB blobs touching
        # the image border. A clipped background is not a clipped cube.
        components=[];blob_records=[]
        for blob in blobs[:4]:
            boxes=component_boxes(img.get_pixel,blob.rect(),WIDTH,HEIGHT)
            blob_records.append({'bbox_px':list(blob.rect()),'pixels':blob.pixels(),'HSV_components':boxes})
            for box in boxes:
                if box['clipped'] and box.get('clipped_sides')!=['bottom']:continue
                # De-duplicate HSV components from overlapping LAB proposals.
                if any(abs(box['bbox_px'][0]-c['bbox_px'][0])<8 and abs(box['bbox_px'][1]-c['bbox_px'][1])<8 and abs(box['bbox_px'][2]-c['bbox_px'][2])<12 and abs(box['bbox_px'][3]-c['bbox_px'][3])<12 for c in components):continue
                components.append(box)
        components.sort(key=lambda c:c['sampled_pixels'],reverse=True)
        attempts=[];result=None
        for box in components[:4]:
            candidate=extract(img.get_pixel,box['bbox_px'],WIDTH,HEIGHT)
            native,native_info=native_rect_faces(img,box['bbox_px'],WIDTH,HEIGHT)
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
        frames.append(result)
        if PREVIEW:
            img.draw_rectangle(tuple(result.get('bbox_px',blobs[0].rect())), color=(0,255,0), thickness=2)
            for face in result['candidates']:
                q=face['quad_px']
                for i in range(4):
                    img.draw_line(int(q[i][0]),int(q[i][1]),int(q[(i+1)%4][0]),int(q[(i+1)%4][1]),color=(255,255,0),thickness=2)
            Display.show_image(img)
        if len(frames)>=3 and matched_three_frames(frames):break
    result = {'schema':'aic.blue_faces.v1','backend':'canmv_native',
              'width':WIDTH,'height':HEIGHT,'hmirror':False,'vflip':False,
              'frames':frames,'frame_count':len(frames),'frame_limit':MAX_FRAMES,
              'temporal_sampling':'bounded burst; select three matching valid faces, transient failures retained',
              'timing_ms':{'initialize_and_warmup':time.ticks_diff(initialized,started),
              'extract_three_frames':time.ticks_diff(time.ticks_ms(),initialized)},
              'robot_commands_sent':False,'image_exported':False}
    save_image_remote=DEBUG_IMAGE_REMOTE or LOCAL_ROI_REMOTE
    if save_image_remote is not None:
        result['debug_image_remote']=save_image_remote
        result['diagnostic_export_requested']=DEBUG_IMAGE_REMOTE is not None
        result['debug_image_frame_index']=len(frames)-1
        result['diagnostic_image_requested']=True
        if DEBUG_IMAGE_ROI or DEBUG_IMAGE_REMOTE is None:
            bb=frames[-1].get('bbox_px')
            if bb is None and frames[-1].get('LAB_blobs'):bb=frames[-1]['LAB_blobs'][0]['bbox_px']
            if bb is not None:
                x,y,w,h=bb;left=max(0,x-20);top=max(0,y-20);right=min(WIDTH,x+w+20);bottom=min(HEIGHT,y+h+20)
                result['debug_image_roi']=[left,top,right-left,bottom-top]
            else:result['debug_image_roi']=[0,0,WIDTH,HEIGHT]
    print('CAPTURE_STAGE send_features')
    payload=json.dumps(result).encode()
    for offset in range(0,len(payload),1024):
        print('FEATURES_PART '+str(offset)+' '+payload[offset:offset+1024].decode())
        time.sleep_ms(5)
    print('FEATURES_HASH '+str(len(payload))+' '+binascii.hexlify(hashlib.sha256(payload).digest()).decode())
    if save_image_remote is not None:
        try:
            print('CAPTURE_STAGE copy_debug_frame')
            # Cache the same final recognition frame on SD. Only explicit
            # debug requests transfer a JPEG during a successful capture.
            debug_img=img.copy(roi=tuple(result['debug_image_roi'])) if result.get('debug_image_roi') else img.copy()
            print('CAPTURE_STAGE stop_sensor')
            sensor.stop()
            sensor_stopped=True
            print('CAPTURE_STAGE save_debug_image')
            try:os.mkdir('/sdcard/pick_tests')
            except OSError:pass
            debug_img.save(save_image_remote,quality=95)
            debug_saved=True
            print('CAPTURE_STAGE image_saved')
        except BaseException as exc:
            if DEBUG_IMAGE_REMOTE is not None:raise
            # Optional diagnostic storage must not invalidate valid coordinates.
            print('DIAGNOSTIC_SAVE_ERROR '+str(exc))

except BaseException as exc:
    error_occurred=True
    print('DETECTION_ERROR '+str(exc))
    sys.print_exception(exc)
finally:
    print('CAPTURE_STAGE cleanup_media')
    if sensor is not None and not sensor_stopped:
        try:sensor.stop()
        except BaseException:pass
    if display_ready:
        try:Display.deinit()
        except BaseException:pass
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    if media_ready:
        try:MediaManager.deinit()
        except BaseException:pass
# JPEG transfer is outside camera/media lifetime, as in the original exporter.
try:
    if DEBUG_IMAGE_REMOTE is not None and debug_saved and not error_occurred:
        print('CAPTURE_STAGE export_debug_image')
        digest=hashlib.sha256();offset=0
        with open(save_image_remote,'rb') as f:
            while True:
                chunk=f.read(2048)
                if not chunk:break
                digest.update(chunk)
                print('DEBUG_IMAGE_PART '+str(offset)+' '+binascii.b2a_base64(chunk).decode().strip())
                offset+=len(chunk)
                time.sleep_ms(15)
        print('DEBUG_IMAGE_HASH '+str(offset)+' '+binascii.hexlify(digest.digest()).decode())
except BaseException as exc:
    print('DETECTION_ERROR '+str(exc))
    sys.print_exception(exc)
finally:
    print('EXPORT_DONE')
