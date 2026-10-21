"""V0 one-shot USBDBG adapter for reusable K230 camera and detector modules."""
import os, time, sys
import ujson as json
import uhashlib as hashlib
import binascii
from blue_detector import BlueCubeDetector, matched_three_frames
from camera import CameraSession

PREVIEW = False
DEBUG_IMAGE_REMOTE = None
LOCAL_ROI_REMOTE = None
DEBUG_IMAGE_ROI = False

def run_capture(config, preview=False, debug_image_remote=None, local_roi_remote=None, debug_image_roi=False):
    VISION_CONFIG = config
    PREVIEW = preview
    DEBUG_IMAGE_REMOTE = debug_image_remote
    LOCAL_ROI_REMOTE = local_roi_remote
    DEBUG_IMAGE_ROI = debug_image_roi
    session = CameraSession(VISION_CONFIG, PREVIEW)
    detector = BlueCubeDetector(VISION_CONFIG)
    WIDTH, HEIGHT = detector.width, detector.height
    MAX_FRAMES = VISION_CONFIG['sampling']['frame_limit']
    started = time.ticks_ms()
    debug_saved = False
    error_occurred = False
    try:
        print('CAPTURE_STAGE initialize')
        session.open()
        initialized = time.ticks_ms()
        print('CAPTURE_STAGE extract_faces')
        frames = []
        for index in range(MAX_FRAMES):
            os.exitpoint()
            img = session.snapshot()
            result = detector.detect_frame(img)
            if result['status'] == 'no_blue_cube':
                frames.append(result)
                continue
            frames.append(result)
            if PREVIEW:
                img.draw_rectangle(tuple(result.get('bbox_px',result['LAB_blobs'][0]['bbox_px'])), color=(0,255,0), thickness=2)
                for face in result['candidates']:
                    q=face['quad_px']
                    for i in range(4):
                        img.draw_line(int(q[i][0]),int(q[i][1]),int(q[(i+1)%4][0]),int(q[(i+1)%4][1]),color=(255,255,0),thickness=2)
                session.show(img)
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
                session.stop()
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
        session.close()
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


if __name__ == '__main__':
    with open('/sdcard/aic/configs/vision/blue_cube.json') as config_file:
        profile = json.load(config_file)
    run_capture(profile, PREVIEW, DEBUG_IMAGE_REMOTE, LOCAL_ROI_REMOTE, DEBUG_IMAGE_ROI)
