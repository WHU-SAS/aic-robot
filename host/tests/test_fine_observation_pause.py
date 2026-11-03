"""Fine callbacks move the scene while the real task has no coarse captures."""
import copy
import unittest

from device.k230.car.blue_tracker import BlueTracker
from device.k230.car.near_control import predict_center
from device.k230.vision.runtime import track_coarse
from host.tests import test_robot_tracking as fixtures


class FineObservationPauseTests(unittest.TestCase):
    def fixture(self, purpose, capability='full'):
        f=fixtures.RobotTrackingTests()
        f.setUp()
        if purpose=='moving':
            f.x=650
        f.fine_slices=40
        f.fine_cost_ms=20
        f.use_real_association_with_synthetic_pose()
        ordinary_track=f.track
        class NativeProjection:
            def blob_geometry(self,box):
                return dict(ground_contact_proxy_Base_mm=self.proxy,
                    observed_to_expected_height_ratio=1.,expected_vertical_height_px=box[3])
        model=NativeProjection()
        tracker=BlueTracker(f.config['tracking'],model)
        class Blob:
            def __init__(self,evidence):self.evidence=evidence
            def rect(self):return self.evidence['bbox_px']
            def cx(self):return self.evidence['center_pixel'][0]
            def cy(self):return self.evidence['center_pixel'][1]
            def pixels(self):return self.evidence['pixels']
        class Image:
            def __init__(self,evidence):self.blob=Blob(evidence)
            def find_blobs(self,*args,**kwargs):return [self.blob]
        def native(point,frame):
            original=ordinary_track(point,frame)
            evidence=original['visual_evidence']
            model.proxy=list(evidence['ground_contact_proxy_Base_mm'])
            return track_coarse(Image(evidence),frame,tracker.detect_frame)
        f.task.track=native
        f.task.reset_tracker=tracker.reset
        projected=f.Fine.advance
        scene_change=[]
        def advance(session):
            projected(session)
            if (not scene_change and session.ticks==3 and
                    f.task.job is not None and f.task.job['purpose']==purpose):
                # This executes independently of capture; the borrowed fine
                # image is unchanged and the next camera input moves40mm.
                f.y+=40
                scene_change.append(dict(at=f.clock.now(),frame=session.frame['frame_id']))
            if (capability=='range' and session.index==0 and
                    session.latest is not None and session.phase is None):
                point=session.latest.pop('cube_center_Base_mm')
                session.latest.update(status='no_complete_face',range_candidate={
                    'schema':'aic.car.side_range.v1','status':'range_candidate',
                    'coordinate_frame':'Base','units':'mm','Base_X_mm':point[0],
                    'raw_center_Base_mm':point,'not_arm_or_parking_authority':True,
                    'selected_faces':copy.deepcopy(session.latest['payload']['side_faces'])})
        f.Fine.advance=advance
        return f,scene_change

    def complete_paused_image(self,f,scene_change,purpose):
        for unused in range(2000):
            f.tick()
            if f.task.complete is not None:
                break
        else:
            self.fail(f.task.snapshot())
        done=copy.deepcopy(f.task.complete)
        self.assertEqual(done['purpose'],purpose)
        self.assertTrue(scene_change)
        self.assertEqual(done['observation']['frame_id'],scene_change[0]['frame'])
        self.assertEqual(f.frame,done['observation']['frame_id'])
        self.assertGreater(done['result_ready_ms']-done['observation']['capture_timestamp_ms'],500)
        self.assertEqual(done['observation']['target_association']['status'],'matched')
        return done

    def assert_stale_image_cannot_command_or_arm(self,f,done):
        command_at=len(f.motor.commands)
        f.tick()
        evidence=f.task.coarse['visual_evidence']
        self.assertGreater(f.task.coarse['frame_id'],done['observation']['frame_id'])
        self.assertEqual(evidence['association_confirmations'],1)
        self.assertFalse(evidence['navigation_confirmed'])
        nonzero=[command for unused,command in f.motor.commands[command_at:]
            if command.split(',')[0] in ('FOLLOW','SPEED','ARC') and
            any(float(value)!=0 for value in command.split(',')[1:])]
        self.assertFalse(nonzero,nonzero)
        self.assertFalse(f.handoffs)
        self.assertFalse(f.arms)
        self.assertIsNone(f.task.memory.last_precise)
        self.assertIsNone(f.task.drive.anchor)

    def test_static_old_full_image_cannot_authorize_arm_after_a_paused_scene_change(self):
        f,moved=self.fixture('static')
        done=self.complete_paused_image(f,moved,'static')
        self.assert_stale_image_cannot_command_or_arm(f,done)

    def test_moving_old_full_and_range_cannot_resume_from_a_paused_scene_change(self):
        for capability in ('full','range'):
            with self.subTest(capability=capability):
                f,moved=self.fixture('moving',capability)
                done=self.complete_paused_image(f,moved,'moving')
                self.assertEqual(done['observation']['target_association']['geometry_capability'],
                    'moving_range' if capability=='range' else 'full_pose')
                self.assert_stale_image_cannot_command_or_arm(f,done)

    def test_current_photo_comparison_excludes_post_exposure_driving(self):
        f=fixtures.RobotTrackingTests()
        f.setUp()
        f.x=400
        source=dict(frame_id=1,capture_timestamp_ms=100,timestamp_ms=100,
                    clock_domain='k230_ticks_ms',image_size_px=[1280,960])
        point=f.point()
        paired,fine=f.track(point,source),f.precise(point,source)
        memory=f.task.memory
        memory.update(paired,motion_state={'run':1})
        current=predict_center(point,[(40,40)],f.config['near_parking']['track_width_mm'],
            f.projection.normal,f.projection.forward_axis_Base,f.projection.motion_origin_Base_mm)
        frame=dict(source,frame_id=2,capture_timestamp_ms=700,timestamp_ms=700)
        memory.update(f.track(current,frame),[(40,40)],motion_state={'run':1})
        odom=f.task.drive.odometry
        for stamp,mm in ((100,0),(700,40),(800,50)):
            counts=round(mm/f.config['mm_per_count'])
            odom.add(dict(encoder_a=counts,encoder_b=-counts),stamp,stamp)
        # 40mm was travelled before the current exposure; another10mm was
        # travelled during its later processing. The cube never moved.
        self.assertEqual(memory.precise_consistency(fine,paired,odom.since(100,700)),
                         'consistent')
        self.assertEqual(memory.precise_consistency(fine,paired,odom.since(100,800)),
                         'changed')


if __name__=='__main__':
    unittest.main()
