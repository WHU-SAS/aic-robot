"""Bottom-search physical ownership with real drive decisions and fake UARTs.

The supplied cues and precise pose are protocol fixtures, not image accuracy
or physical travel evidence. Existing nominal encoder/settle math is exercised.
"""
import unittest

from host.tests import test_car_tracking_drive as fixtures


class BottomSearchReverseTests(unittest.TestCase):
    def setUp(self):
        self.f=fixtures.TrackingDriveTests()
        self.f.setUp()
        self.f.config['tracking']['confirmation_frames']=1
        self.drive=self.f.drive

    def step(self,now,run=False,travel=0,coarse=None,**kwargs):
        return self.f.update(now,run=run,travel=travel,
            coarse=self.f.missing(now) if coarse is None else coarse,**kwargs)

    def begin(self):
        self.step(1000,target_event={'event':'exited','last_disappearance_direction':'bottom'})
        result=self.step(1050)
        self.assertEqual(result['command'],'FOLLOW,-15,-15')
        self.assertEqual(self.drive._active_action,'bottom_reverse')
        return list(self.drive._bottom_reverse_budget['reverse_start_counts'])

    def recovered(self,now,x=650,weak=False):
        coarse=self.f.coarse(now,x)
        event={}
        if weak:
            coarse['visual_evidence'].update(distance_valid=False,navigation_confirmed=False,
                reason='inconsistent_with_20mm_height')
            event=dict(precise_candidate=True,target_valid=False,visible=True,associated=True)
        return coarse,event

    def test_missing_partial_tiny_ambiguous_and_stale_views_keep_the_same_reverse(self):
        origin=self.begin()
        for index,kind in enumerate(('missing','partial','tiny','ambiguous','stale')):
            now=1150+index*100
            cue=self.f.missing(now)
            if kind=='partial':cue=self.f.clipped_near(now)
            elif kind=='tiny':
                cue=self.f.weak_cue(now,1)
                cue['visual_evidence'].update(bbox_px=[640,450,1,1],pixels=1,
                    center_pixel=[640,450],reason='invalid_or_tiny_blob')
            elif kind=='ambiguous':
                cue=self.f.coarse(now)
                cue['visual_evidence']['ambiguous']=True
            elif kind=='stale':cue=self.f.coarse(now-600)
            result=self.step(now,run=True,travel=-1-index,coarse=cue,
                target_event={'precise_candidate':True} if kind in ('tiny','ambiguous') else {})
            self.assertEqual(result['command'],'HB',(kind,result))
            self.assertIsNone(result['request_fine'])
            self.assertEqual(self.drive._bottom_reverse_budget['reverse_start_counts'],origin)
            self.assertEqual(self.drive._active_action,'bottom_reverse')

    def test_whole_view_with_or_without_coarse_distance_stops_before_static(self):
        for weak in (False,True):
            with self.subTest(coarse_distance_available=not weak):
                self.setUp()
                self.begin()
                cue,event=self.recovered(1150,weak=weak)
                if weak:
                    cue['visual_evidence']['associated']=False
                    event.update(precise_candidate=False,associated=False)
                result=self.step(1150,True,-2,coarse=cue,target_event=event)
                self.assertEqual(result['command'],'STOP')
                if weak:self.assertTrue(result['reset_tracker'])
                self.assertIsNone(result['request_fine'])
                self.assertTrue(self.drive._bottom_static_pending)
                cue,event=self.recovered(1250,weak=weak)
                result=self.step(1250,False,-2,reason=1,coarse=cue,target_event=event)
                self.assertIsNone(result['request_fine'])
                cue,event=self.recovered(1300,weak=weak)
                result=self.step(1300,False,-2,reason=1,coarse=cue,target_event=event)
                self.assertEqual(result['request_fine'],'static')
                self.assertIsNone(result['command'])
                self.assertGreater(self.drive._fine_request['capture_timestamp_ms'],1150)

    def test_old_pre_stop_whole_frame_is_not_reused_for_static_capture(self):
        self.begin()
        old,event=self.recovered(1150)
        self.step(1150,True,-2,coarse=old,target_event=event)
        self.step(1250,False,-2,reason=1,coarse=old)
        result=self.step(1300,False,-2,reason=1,coarse=old)
        self.assertEqual(result['status'],'waiting_for_post_stop_view')
        self.assertIsNone(result['request_fine'])
        fresh,event=self.recovered(1310)
        self.assertEqual(self.step(1310,False,-2,reason=1,coarse=fresh)['request_fine'],'static')

    def test_deadline_manual_resume_and_versions_do_not_renew_100mm_budget(self):
        origin=self.begin()
        self.step(2000,True,-40)
        self.assertIsNone(self.step(21050,False,-45,reason=3)['command'])
        self.assertEqual(self.step(21100,False,-45,reason=3)['command'],'FOLLOW,-15,-15')
        paused=self.step(21200,False,-46,reason=4)
        self.assertEqual(paused['status'],'motion_wait')
        self.drive.resume_motion()
        self.assertEqual(self.step(21250,False,-46,reason=4)['command'],'FOLLOW,-15,-15')
        for version in (2,3,4):
            now=21300+version*100
            result=self.step(now,True,-50-version,target_version=version,
                target_event={'event':'exited','last_disappearance_direction':'bottom'})
            self.assertEqual(result['command'],'HB')
            self.assertEqual(self.drive._bottom_reverse_budget['reverse_start_counts'],origin)
        stopped=self.step(22000,True,-100,target_version=4)
        self.assertEqual(stopped['command'],'STOP')
        self.assertTrue(self.drive._bottom_reverse_budget['reverse_exhausted'])
        for now in (22100,22150,22200,23000):
            waiting=self.step(now,False,-100,reason=1,target_version=4)
            self.assertIsNone(waiting['command'])
            self.assertEqual(waiting['status'],'bottom_reverse_limit')
            self.assertIsNone(waiting['request_fine'])
            self.assertEqual(self.drive._bottom_reverse_budget['reverse_start_counts'],origin)

    def test_whole_after_budget_limit_still_goes_to_new_static_not_forward(self):
        self.begin()
        self.assertEqual(self.step(1150,True,-100)['command'],'STOP')
        self.step(1250,False,-100,reason=1)
        self.step(1300,False,-100,reason=1)
        cue,event=self.recovered(1400)
        seen=self.step(1400,False,-100,reason=1,coarse=cue,target_event=event)
        self.assertIsNone(seen['command'])
        self.assertTrue(self.drive._bottom_static_pending)
        cue,event=self.recovered(1500)
        fine=self.step(1500,False,-100,reason=1,coarse=cue,target_event=event)
        self.assertEqual(fine['request_fine'],'static')
        self.assertIsNone(fine['command'])

    def test_only_new_static_measurement_releases_old_budget(self):
        origin=self.begin()
        cue,event=self.recovered(1150)
        self.step(1150,True,-25,coarse=cue)
        self.step(1250,False,-25,reason=1,coarse=self.f.coarse(1250))
        request=self.step(1300,False,-25,reason=1,coarse=self.f.coarse(1300))
        self.assertEqual(request['request_fine'],'static')
        self.assertEqual(self.drive._bottom_reverse_budget['reverse_start_counts'],origin)
        complete=self.f.precise(1300,self.f.target,version=1)
        accepted=self.step(1400,False,-25,reason=1,
            coarse=self.f.coarse(1400,self.f.target),precise=complete)
        self.assertTrue(accepted['allow_arm'])
        self.assertIsNone(self.drive._bottom_reverse_budget)
        self.step(1500,False,-25,reason=1,target_version=2,
            target_event={'event':'exited','last_disappearance_direction':'bottom'})
        next_episode=self.step(1550,False,-25,reason=1,target_version=2)
        self.assertEqual(next_episode['command'],'FOLLOW,-15,-15')
        self.assertNotEqual(self.drive._bottom_reverse_budget['reverse_start_counts'],origin)

    def test_sticky_bottom_direction_alone_does_not_authorize_new_blind_reverse(self):
        self.step(1000,target_event={'search':True,'last_disappearance_direction':'bottom'})
        self.assertIsNone(self.drive._bottom_reverse_budget)
        result=self.step(1050)
        self.assertNotEqual(result['command'],'FOLLOW,-15,-15')
        self.assertNotEqual(result['command'],'ARC,-10,-10')

    def test_current_bottom_exit_overrides_rotation_without_resetting_episode(self):
        self.step(1000,target_event={'search':True,'last_disappearance_direction':'left'})
        self.assertEqual(self.step(1050)['command'],'FOLLOW,-20,20')
        switch=self.step(1150,True,target_version=2,
            target_event={'event':'exited','last_disappearance_direction':'bottom'})
        self.assertEqual(switch['command'],'STOP')
        self.step(1250,False,reason=1,target_version=2)
        self.assertEqual(self.step(1300,False,reason=1,target_version=2)['command'],'FOLLOW,-15,-15')
        origin=list(self.drive._bottom_reverse_budget['reverse_start_counts'])
        repeated=self.step(1400,True,-1,target_version=3,
            target_event={'event':'exited','last_disappearance_direction':'bottom'})
        self.assertEqual(repeated['command'],'HB')
        self.assertEqual(self.drive._bottom_reverse_budget['reverse_start_counts'],origin)

    def test_existing_near_bottom_entry_also_holds_at_limit_until_new_static(self):
        travel,unused=self.f.failed_static_after_late_moving_fine(bottom=True)
        origin=list(self.drive._bottom_reverse_budget['reverse_start_counts'])
        limit=self.step(12450,True,travel-99,coarse=self.f.clipped_near(12450))
        self.assertEqual(limit['command'],'STOP')
        self.assertTrue(self.drive._bottom_reverse_budget['reverse_exhausted'])
        for now in (12550,12600,13000):
            wait=self.step(now,False,travel-99,reason=1,coarse=self.f.clipped_near(now))
            self.assertIsNone(wait['command'])
            self.assertEqual(wait['status'],'bottom_reverse_limit')
        cue,event=self.recovered(13100,weak=True)
        self.step(13100,False,travel-99,reason=1,coarse=cue,target_event=event)
        cue,event=self.recovered(13200,weak=True)
        fine=self.step(13200,False,travel-99,reason=1,coarse=cue,target_event=event)
        self.assertEqual(fine['request_fine'],'static')
        self.assertIsNone(fine['command'])
        self.assertEqual(self.drive._bottom_reverse_budget['reverse_start_counts'],origin)

    def test_near_reverse_can_transfer_to_global_bottom_without_stopping_or_new_budget(self):
        travel,unused=self.f.failed_static_after_late_moving_fine(bottom=True)
        origin=list(self.drive._bottom_reverse_budget['reverse_start_counts'])
        transfer=self.step(6000,True,travel-2,target_version=2,
            target_event={'event':'exited','last_disappearance_direction':'bottom'})
        self.assertEqual(transfer['command'],'HB')
        self.assertEqual(self.drive._active_action,'bottom_reverse')
        self.assertEqual(self.drive._bottom_reverse_budget['reverse_start_counts'],origin)


if __name__=='__main__':
    unittest.main()
