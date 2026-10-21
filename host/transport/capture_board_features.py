"""K230 coordinate capture with optional same-frame debug image, no servo access."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=False)
import base64,hashlib,json,struct,time
from pathlib import Path
from download_calibration import CameraLink,PORT
from host.transport.board_vision_source import build_capture_script

def capture(output,port=PORT,debug_image=False,debug_roi=False):
    if debug_roi:debug_image=True
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    remote='/sdcard/pick_tests/'+output.name+'_board_debug.jpg' if debug_image else None
    script=build_capture_script(debug_image_remote=remote,
        local_roi_remote='/sdcard/pick_tests/'+output.name+'_cached_roi.jpg',
        debug_image_roi=bool(debug_roi))
    link=CameraLink(port);logs=[];features=None;chunks=bytearray();image_chunks=bytearray()
    expected_hash=None;expected_size=None;image_hash=None;image_size=None
    started=time.monotonic();stage='starting';firmware=None;image_name='frame.jpg'
    stage_names={'initialize':'初始化相机','extract_faces':'采样并匹配三帧角点（最多9帧）','send_features':'回传角点',
                 'copy_debug_frame':'复制同帧诊断图','stop_sensor':'停止相机采集',
                 'save_debug_image':'保存诊断图到SD卡','image_saved':'诊断图已保存',
                 'cleanup_media':'释放相机资源','export_debug_image':'传输诊断图'}
    def status(error=None):
        value={'elapsed_s':time.monotonic()-started,'last_stage':stage,'feature_bytes':len(chunks),
               'features_validated':features is not None,'image_bytes_received':len(image_chunks),
               'image_size_expected':image_size,'error':error}
        (output/'transfer_status.json').write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
    def save_features():
        nonlocal features,image_name
        if expected_size!=len(chunks) or expected_hash!=hashlib.sha256(chunks).hexdigest():raise RuntimeError('K230角点传输长度或校验不符')
        candidate=json.loads(chunks)
        if candidate.get('schema')!='aic.blue_faces.v1' or candidate.get('backend')!='canmv_native':raise RuntimeError('K230 feature schema mismatch')
        if candidate.get('robot_commands_sent') is not False:raise RuntimeError('Unexpected camera protocol')
        features=candidate
        image_name='frame_roi.jpg' if features.get('debug_image_roi') else 'frame.jpg'
        (output/'board_features.json').write_text(json.dumps(features,ensure_ascii=False,indent=2)+'\n')
        status()
    try:
        link.command(0x80,12);firmware=list(struct.unpack('<III',link.read(12)))
        for line in link.execute(script,timeout=90):
            if line.startswith(b'CAPTURE_STAGE '):
                stage=line[len(b'CAPTURE_STAGE '):].decode();logs.append(stage);status()
                if debug_image:print('诊断：'+stage_names.get(stage,stage),flush=True)
            elif line.startswith(b'DEBUG_IMAGE_PART '):
                _,offset,chunk=line.split(b' ',2)
                if int(offset)!=len(image_chunks):raise RuntimeError('诊断图像分块丢失或重复')
                image_chunks.extend(base64.b64decode(chunk,validate=True))
            elif line.startswith(b'DEBUG_IMAGE_HASH '):
                _,size,digest=line.split();image_size=int(size);image_hash=digest.decode()
                if len(image_chunks)!=image_size or image_hash!=hashlib.sha256(image_chunks).hexdigest():raise RuntimeError('诊断图像长度或校验不符')
                temporary=output/'frame.transfer';temporary.write_bytes(image_chunks);temporary.replace(output/image_name);status()
            elif line.startswith(b'FEATURES_PART '):
                _,offset,chunk=line.split(b' ',2)
                if int(offset)!=len(chunks):raise RuntimeError('K230角点分块丢失或重复')
                chunks.extend(chunk)
            elif line.startswith(b'FEATURES_HASH '):
                _,size,digest=line.split();expected_size=int(size);expected_hash=digest.decode();save_features()
            elif line.startswith(b'FEATURES '):
                if features is not None:raise RuntimeError('Duplicate K230 feature packet')
                chunks.extend(line[len(b'FEATURES '):]);expected_size=len(chunks);expected_hash=hashlib.sha256(chunks).hexdigest();save_features()
            else:
                message=line.decode(errors='replace');logs.append(message)
                if 'DETECTION_ERROR' in message or 'Traceback' in message:raise RuntimeError('K230识别/诊断失败：'+message)
        if features is None:raise RuntimeError('K230未返回角点数据')
        if debug_image and not (output/image_name).exists():raise RuntimeError('角点已保存，但K230没有完整导出诊断图')
        if not debug_image and image_chunks:raise RuntimeError('Unexpected image in coordinate-only capture')
        details={'backend':'canmv_native','firmware':firmware,'elapsed_s':time.monotonic()-started,
                 'script_sha256':hashlib.sha256(script.encode()).hexdigest(),'image_exported':bool(debug_image),
                 'diagnostic_image_bytes':len(image_chunks),'diagnostic_image_file':image_name if debug_image else None,'diagnostic_image_roi':features.get('debug_image_roi'),'diagnostic_image_frame_index':features.get('debug_image_frame_index') if debug_image else None,
                 'same_frame_roi_remote':features.get('debug_image_remote'),'same_frame_roi':features.get('debug_image_roi'),'same_frame_roi_index':features.get('debug_image_frame_index'),
                 'same_frame_roi_sd_saved':stage=='image_saved' or 'image_saved' in logs,
                 'feature_payload_bytes':len(chunks),'board_timing_ms':features.get('timing_ms'),'robot_commands_sent':False}
        (output/'capture.json').write_text(json.dumps(details,indent=2)+'\n');status();return details
    except BaseException as exc:
        if chunks and features is None:(output/'features.partial').write_bytes(chunks)
        if image_chunks and not (output/image_name).exists():(output/'frame.partial').write_bytes(image_chunks)
        status(repr(exc))
        if isinstance(exc,TimeoutError):
            raise TimeoutError('K230诊断超时；最后阶段：'+stage_names.get(stage,stage)+'；角点'+('已保存' if features else '尚未收到')+'；已收到图像'+str(len(image_chunks))+'字节') from exc
        raise
    finally:
        (output/'camera_log.txt').write_text('\n'.join(logs)+'\n');link.port.close()


def fetch_saved_diagnostic(output,port=PORT):
    """Read the cached recognition ROI after PC localization fails; no recapture.
    Uses only the camera's SD file export, never opens the servo port.
    """
    output=Path(output)
    if (output/'frame_roi.jpg').exists() or (output/'frame.jpg').exists():return {'status':'already_present'}
    metadata=json.loads((output/'board_features.json').read_text());remote=metadata.get('debug_image_remote')
    expected='/sdcard/pick_tests/'+output.name+'_cached_roi.jpg'
    if remote!=expected:return {'status':'not_available','reason':'No matching cached recognition frame'}
    script="""import time,binascii
import uhashlib as hashlib
try:
 digest=hashlib.sha256();offset=0
 with open(%r,'rb') as f:
  while True:
   chunk=f.read(2048)
   if not chunk:break
   digest.update(chunk)
   print('DATA '+str(offset)+' '+binascii.b2a_base64(chunk).decode().strip())
   offset+=len(chunk)
   time.sleep_ms(15)
 print('HASH '+str(offset)+' '+binascii.hexlify(digest.digest()).decode())
except BaseException as exc:
 print('DIAGNOSTIC_ERROR '+str(exc))
finally:
 print('EXPORT_DONE')
"""%remote
    link=CameraLink(port);data=bytearray();size=None;digest=None;started=time.monotonic()
    try:
        for line in link.execute(script,timeout=45):
            if line.startswith(b'DATA '):
                _,offset,chunk=line.split(b' ',2)
                if int(offset)!=len(data):raise RuntimeError('Cached ROI chunks lost or repeated')
                data.extend(base64.b64decode(chunk,validate=True))
            elif line.startswith(b'HASH '):
                _,size,digest=line.split();size=int(size);digest=digest.decode()
            elif line.startswith(b'DIAGNOSTIC_ERROR '):raise RuntimeError(line.decode(errors='replace'))
        if size!=len(data) or digest!=hashlib.sha256(data).hexdigest():raise RuntimeError('Cached ROI size/hash mismatch')
        temporary=output/'frame.transfer';temporary.write_bytes(data);temporary.replace(output/'frame_roi.jpg')
        result={'status':'saved','remote':remote,'file':'frame_roi.jpg','bytes':size,'sha256':digest,
                'frame_index':metadata.get('debug_image_frame_index'),'roi':metadata.get('debug_image_roi'),
                'elapsed_s':time.monotonic()-started,'same_recognition_frame':True,'recaptured':False,'robot_commands_sent':False}
        (output/'failure_diagnostic.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        return result
    finally:link.port.close()
