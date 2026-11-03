"""Soft fine/coarse work accounting through real tracking state machines.

The camera, fine solver and UART are protocol doubles with explicit wall-clock
costs. These tests establish scheduling and stale-result authority, not K230
performance, vision accuracy or physical driving success.
"""
import copy
import unittest
from unittest.mock import patch

from host.tests import test_fine_observation_pause as paused
from host.tests import test_global_bottom_recovery_task as bottom


class CostScene:
    def __init__(self, test, purpose='static', ratio=.1, slices=90,
                 start_ms=40, advance_ms=20, capture_ms=10, track_ms=20,
                 classify_ms=10, base=None):
        self.test=test
        self.helper=paused.FineObservationPauseTests()
        build=paused.robot_bundle
        def configured_bundle(*args,**kwargs):
            bundle=build(*args,**kwargs)
            bundle[0]['car']['tracking_task'].update(
                fine_observation_overhead_fraction=ratio,fine_observation_reserve_ms=60)
            return bundle
        if base is None:
            with patch.object(paused,'robot_bundle',side_effect=configured_bundle):
                self.f,unused=self.helper.fixture(purpose,move_during_fine=False,
                                                 overhead_fraction=ratio)
        else:
            self.f=base
        f=self.f
        f.config['tracking_task']['fine_observation_overhead_fraction']=ratio
        f.config['tracking_task']['fine_observation_reserve_ms']=60
        f.fine_slices,f.fine_cost_ms=slices,advance_ms
        self.costs=dict(capture=capture_ms,track=track_ms,classify=classify_ms)
        self.extra=[];self.calls=[];self.work={};self.starts=[];self.drive_calls=[]
        self.on_extra=None
        capture=f.task.capture
        def costly_capture():
            job=f.task.job
            row=None
            if job is not None:
                row=dict(at=f.clock.now(),fine=job['session'],
                    work=self.work[id(job['session'])],commands=len(f.motor.commands),
                    paired=copy.deepcopy(job['paired_coarse']),
                    bbox=copy.deepcopy(job['localization_bbox_px']),
                    image_frame=job['session'].frame['frame_id'])
                self.extra.append(row)
                if self.on_extra:self.on_extra(row)
            image,frame=capture()
            if row is not None:row['frame']=dict(frame)
            f.clock.advance(self.costs['capture'])
            if row is not None:
                row['capture_cost_ms']=self.costs['capture']
                row['capture_commands_after']=len(f.motor.commands)
            self.calls.append(('capture',f.clock.now(),job))
            return image,frame
        f.task.capture=costly_capture
        track=f.task.track
        def costly_track(image,frame):
            before=len(f.motor.commands)
            began=f.clock.now()
            result=track(image,frame)
            f.clock.advance(self.costs['track'])
            if self.extra and self.extra[-1].get('frame',{}).get('frame_id')==frame['frame_id']:
                self.extra[-1].update(track_cost_ms=f.clock.now()-began,
                    track_commands_before=before,track_commands_after=len(f.motor.commands))
            self.calls.append(('track',f.clock.now(),f.task.job))
            return result
        f.task.track=costly_track
        update=f.task.memory.update
        def costly_classify(*args,**kwargs):
            before=len(f.motor.commands)
            began=f.clock.now()
            result=update(*args,**kwargs)
            f.clock.advance(self.costs['classify'])
            self.calls.append(('classify',f.clock.now(),f.task.job))
            frame=args[0].get('frame_id')
            row=next((row for row in reversed(self.extra)
                      if row.get('frame',{}).get('frame_id')==frame),None)
            if row is not None:
                row.update(end=f.clock.now(),version=f.task.memory.version,
                    target=copy.deepcopy(result),classify_commands_before=before,
                    classify_commands_after=len(f.motor.commands),
                    classify_cost_ms=f.clock.now()-began)
            return result
        f.task.memory.update=costly_classify
        drive_update=f.task.drive.update
        def record_drive(*args,**kwargs):
            self.drive_calls.append(dict(at=f.clock.now(),job=f.task.job))
            return drive_update(*args,**kwargs)
        f.task.drive.update=record_drive
        start=f.Fine.start
        def costly_start(session,image,frame,bbox):
            self.work[id(session)]=0
            began=f.clock.now()
            f.clock.advance(start_ms)
            result=start(session,image,frame,bbox)
            self.work[id(session)]+=f.clock.now()-began
            self.starts.append(dict(session=session,at=began,end=f.clock.now(),
                                    frame=dict(frame),point=copy.deepcopy(image)))
            return result
        f.Fine.start=costly_start
        advance=f.Fine.advance
        def costly_advance(session):
            began=f.clock.now()
            result=advance(session)
            self.work[id(session)]+=f.clock.now()-began
            return result
        f.Fine.advance=costly_advance

    def until(self,predicate,limit=5000):
        f=self.f
        for unused in range(limit):
            f.tick()
            self.test.assertIsNone(f.task.fault,f.task.snapshot())
            if predicate():return
        self.test.fail(dict(phase=f.task.snapshot()['phase'],frame=f.frame,
            extras=len(self.extra),metrics={key:f.task.metrics[key]
                for key in ('fine_started','fine_cancelled','target_changes')},
            commands=f.motor.commands[-8:]))

    def first_complete(self):
        self.until(lambda:self.f.task.complete is not None)
        return copy.deepcopy(self.f.task.complete)


class FineObservationBudgetTests(unittest.TestCase):
    def synthetic_benchmark(self,costs=(20,40,50,60),start_ms=40,slices=90):
        """Paired fixed-target runs; report wall costs without a hard SLA claim."""
        rows=[]
        for coarse_ms in costs:
            pair=[]
            for ratio in (0,.1):
                # Split the actual coarse work across capture, native tracker
                # and classification. UART time is ordinary pre-existing work.
                scene=CostScene(self,purpose='moving',ratio=ratio,slices=slices,start_ms=start_ms,
                    capture_ms=5,track_ms=coarse_ms-10,classify_ms=5)
                f=scene.f
                f.motor.request_cost_ms=24
                # Match run_tracking: pending fine work has zero idle delay.
                # The general test fixture's fixed5ms tick would dilute the
                # measured overhead with sleep absent from production.
                for unused in range(10000):
                    f.task.step()
                    self.assertIsNone(f.task.fault)
                    if f.task.complete is not None:break
                    f.clock.advance(f.task.idle_delay_ms())
                else:self.fail('Synthetic benchmark never completed fine work')
                fine=f.created_fines[0]
                started,ready=scene.starts[0]['at'],fine.latest_ready_ms
                summary=f.task.metrics['fine_observation_jobs'][0]
                heartbeats=sum(command=='HB' and started<=at<ready
                    for at,command in f.motor.commands)
                other=[command for at,command in f.motor.commands
                       if started<=at<ready and command!='HB']
                self.assertEqual(other,[])
                pair.append(dict(ratio=ratio,wall_ms=ready-started,
                    fine_ms=summary['fine_compute_ms'],
                    coarse_ms=summary['observation_ms'],
                    observation_count=summary['observation_count'],
                    hb_count=heartbeats,hb_ms=heartbeats*24))
            pure,observed=pair
            self.assertEqual(pure['fine_ms'],observed['fine_ms'])
            self.assertEqual(pure['coarse_ms'],0)
            self.assertEqual(observed['fine_ms'],start_ms+slices*20)
            self.assertGreater(observed['observation_count'],0)
            self.assertLessEqual(observed['coarse_ms'],.1*observed['fine_ms'])
            delta=observed['wall_ms']-pure['wall_ms']
            rows.append(dict(coarse_cost_ms=coarse_ms,uart_cost_ms=24,
                pure=pure,observed=observed,wall_delta_ms=delta,
                wall_overhead_fraction=delta/pure['wall_ms'],
                accounted_fraction=observed['coarse_ms']/observed['fine_ms'],
                extra_hb_ms=observed['hb_ms']-pure['hb_ms'],
                remaining_delta_ms=delta-observed['coarse_ms']-
                    (observed['hb_ms']-pure['hb_ms'])))
        return rows

    def test_paired_synthetic_wall_benchmark_accounts_for_normal_24ms_heartbeat_cost(self):
        rows=self.synthetic_benchmark()
        self.assertEqual([row['coarse_cost_ms'] for row in rows],[20,40,50,60])
        for row in rows:
            self.assertGreater(row['pure']['hb_count'],0)
            self.assertGreater(row['observed']['hb_count'],0)
            self.assertGreaterEqual(row['wall_delta_ms'],0)

    def test_heartbeat_phase_cost_is_visible_even_when_coarse_accounting_is_under_ten_percent(self):
        row=self.synthetic_benchmark(costs=(60,),start_ms=20,slices=90)[0]
        self.assertLessEqual(row['accounted_fraction'],.1)
        self.assertEqual(row['remaining_delta_ms'],0)
        self.assertEqual(row['wall_delta_ms'],row['observed']['coarse_ms']+row['extra_hb_ms'])
        self.assertGreater(row['extra_hb_ms'],0)
        # Current fixture:1820ms fine,175ms extra observation and one more24ms
        # original HB;1988→2187ms is10.0101% wall overhead despite9.6154% C/F.
        # Deliberately no <=10% wall assertion: this is the documented soft
        # budget boundary, not a native-call or whole-pipeline latency SLA.

    def test_ratio_zero_keeps_pure_fine_pause_and_fresh_status_before_acceptance(self):
        s=CostScene(self,ratio=0)
        done=s.first_complete();f=s.f
        self.assertEqual(s.extra,[])
        self.assertEqual(f.frame,done['observation']['frame_id'])
        self.assertEqual(s.work[id(f.created_fines[0])],40+90*20)
        before=len(f.motor.commands)
        f.tick()
        self.assertGreater(f.task.coarse['frame_id'],done['observation']['frame_id'])
        self.assertIn('STATUS',[command for unused,command in f.motor.commands[before:]])

    def test_budget_insufficient_for_reserved_observation_does_not_capture(self):
        s=CostScene(self,slices=20,start_ms=40)
        done=s.first_complete();f=s.f
        self.assertEqual(s.work[id(f.created_fines[0])],440)
        self.assertEqual(s.extra,[])
        self.assertEqual(f.frame,done['observation']['frame_id'])

    def test_actual_start_and_advance_work_fund_only_finite_lightweight_observations(self):
        s=CostScene(self,slices=90,start_ms=200)
        done=s.first_complete();f=s.f
        first=f.created_fines[0]
        self.assertEqual(s.work[id(first)],2000)
        self.assertTrue(s.extra)
        self.assertLessEqual(len(s.extra),4)
        self.assertGreaterEqual(s.extra[0]['work'],600)
        spent=0
        for row in s.extra:
            self.assertIs(row['fine'],first)
            self.assertLessEqual(spent+60,row['work']*.1)
            cost=sum(row[key] for key in ('capture_cost_ms','track_cost_ms','classify_cost_ms'))
            self.assertEqual(cost,40)
            self.assertEqual(row['commands'],row['capture_commands_after'])
            self.assertEqual(row['track_commands_before'],row['track_commands_after'])
            self.assertEqual(row['classify_commands_before'],row['classify_commands_after'])
            self.assertEqual(row['image_frame'],done['observation']['frame_id'])
            self.assertEqual(row['paired']['frame_id'],done['observation']['frame_id'])
            self.assertEqual(row['bbox'],done['localization_bbox_px'])
            spent+=cost
        self.assertEqual(done['paired_coarse']['frame_id'],done['observation']['frame_id'])
        self.assertFalse(any(row['job'] is not None for row in s.drive_calls))
        summary=f.task.metrics['fine_observation_jobs'][-1]
        self.assertEqual(summary['fine_compute_ms'],s.work[id(first)])
        self.assertEqual(summary['observation_count'],len(s.extra))
        self.assertEqual(summary['observation_ms'],40*len(s.extra))
        self.assertEqual(summary['observation_reserve_ms'],60)
        self.assertEqual(summary['observation_overruns'],0)
        self.assertEqual(done['observation']['capture_timestamp_ms'],
                         s.starts[0]['frame']['capture_timestamp_ms'])
        self.assertEqual(len(f.task.evidence),1)
        self.assertEqual(f.task.evidence[0]['frame']['paired_coarse']['frame_id'],
                         done['observation']['frame_id'])
        last_extra=max(row['frame']['frame_id'] for row in s.extra)
        commands_at=len(f.motor.commands)
        f.tick()
        self.assertGreater(f.task.coarse['frame_id'],last_extra)
        self.assertGreater(f.task.coarse['frame_id'],done['observation']['frame_id'])
        self.assertIn('STATUS',[command for unused,command in f.motor.commands[commands_at:]])

    def test_single_native_decode_or_next_call_may_overrun_the_soft_slice(self):
        s=CostScene(self,slices=3,start_ms=310,advance_ms=275)
        f=s.f
        s.until(lambda:bool(f.created_fines))
        fine=f.created_fines[0]
        self.assertGreaterEqual(s.work[id(fine)],585)
        s.first_complete()
        self.assertEqual(fine.ticks,3)
        self.assertEqual(s.work[id(fine)],310+3*275)
        self.assertTrue(all(row['work'] in (585,860) for row in s.extra),s.extra)

    def test_real_observation_overrun_raises_reserve_and_reduces_later_frequency(self):
        normal=CostScene(self,slices=90,start_ms=40)
        normal.first_complete()
        slow=CostScene(self,slices=90,start_ms=40,capture_ms=40,track_ms=50,classify_ms=30)
        slow.first_complete()
        self.assertTrue(slow.extra)
        self.assertEqual(sum(slow.extra[0][key] for key in
            ('capture_cost_ms','track_cost_ms','classify_cost_ms')),120)
        self.assertLess(len(slow.extra),len(normal.extra))
        self.assertEqual(len(slow.extra),1)
        self.assertIsNone(slow.f.task.fault)
        summary=slow.f.task.metrics['fine_observation_jobs'][-1]
        self.assertGreaterEqual(summary['observation_overruns'],1)
        self.assertGreaterEqual(summary['observation_reserve_ms'],120)
        self.assertEqual(summary['observation_ms'],120)

    def test_moving_visible_jump_discards_old_fine_and_keeps_following_without_stop(self):
        s=CostScene(self,purpose='moving',slices=110)
        f=s.f
        s.until(lambda:f.task.job is not None)
        old=f.task.job['session'].frame['frame_id']
        at=len(f.motor.commands)
        def move_once(row):
            # At the moving-fine distance this clears the existing jump gate
            # while keeping the complete native colour region in view.
            if len(s.extra)==1:f.y+=60
        s.on_extra=move_once
        s.until(lambda:f.task.metrics['fine_cancelled']>0)
        self.assertTrue(s.extra)
        self.assertNotIn(old,f.bind_attempts)
        self.assertIsNone(f.task.drive.anchor)
        self.assertIsNone(f.task.memory.last_precise)
        self.assertFalse(f.task.drive._searching)
        self.assertFalse(f.handoffs)
        s.until(lambda:len(f.created_fines)>=2)
        self.assertNotIn('STOP',[command for unused,command in f.motor.commands[at:]])
        self.assertEqual(f.motor.run,1)
        self.assertEqual(f.fine_start_states[-1]['purpose'],'moving')
        self.assertEqual(f.fine_start_states[-1]['run'],1)
        self.assertGreater(f.created_fines[-1].frame['frame_id'],old)

    def test_static_confirmed_move_cancels_original_image_without_arm_handoff(self):
        # Cheap observations can satisfy the existing two-frame movement
        # confirmation window without changing that independent policy.
        s=CostScene(self,slices=140,capture_ms=5,track_ms=10,classify_ms=5)
        f=s.f
        s.until(lambda:f.task.job is not None)
        old=f.task.job['session'].frame['frame_id']
        version=f.task.memory.version
        def move_once(row):
            if len(s.extra)==1:f.y+=40
        s.on_extra=move_once
        s.until(lambda:f.task.memory.version>version)
        self.assertGreaterEqual(len(s.extra),2)
        self.assertGreater(f.task.metrics['fine_cancelled'],0)
        self.assertNotIn(old,f.bind_attempts)
        self.assertFalse(f.handoffs)
        self.assertFalse(f.arms)
        self.assertTrue(f.task.memory.last_precise is None or
                        f.task.memory.last_precise['frame_id']!=old)
        self.assertTrue(f.task.drive.anchor is None or f.task.drive.anchor['frame_id']!=old)

    def test_unknown_extra_observation_is_not_reported_as_target_motion(self):
        s=CostScene(self,slices=90)
        f=s.f
        s.until(lambda:f.task.job is not None)
        version=f.task.memory.version
        original=f.task.job['session']
        def hide_once(row):
            if len(s.extra)==1:f.hide_frames=1
        s.on_extra=hide_once
        s.until(lambda:bool(s.extra) and 'target' in s.extra[0])
        self.assertFalse(s.extra[0]['target']['confirmed_moved'])
        self.assertEqual(f.task.memory.version,version)
        self.assertEqual(f.task.metrics['target_changes'],0)
        self.assertEqual(f.task.metrics['fine_cancelled'],0)
        self.assertIs(f.task.job['session'],original)
        self.assertFalse(f.handoffs)

    def test_sparse_static_observations_keep_original_confirmation_policy_and_freshly_reject_old_image(self):
        s=CostScene(self,slices=90,capture_ms=10,track_ms=30,classify_ms=10)
        f=s.f
        def move_once(row):
            if len(s.extra)==1:f.y+=40
        s.on_extra=move_once
        done=s.first_complete()
        old=done['observation']['frame_id']
        self.assertGreaterEqual(len(s.extra),2)
        self.assertGreater(s.extra[1]['frame']['capture_timestamp_ms']-
                           s.extra[0]['frame']['capture_timestamp_ms'],
                           f.task.memory._motion_window_ms)
        classified=[row for row in s.extra if 'target' in row]
        self.assertTrue(classified)
        self.assertFalse(any(row['target']['confirmed_moved'] for row in classified))
        self.assertEqual(f.task.metrics['fine_cancelled'],0)
        f.tick()
        self.assertNotIn(old,f.bind_attempts)
        self.assertFalse(f.handoffs)
        self.assertFalse(f.arms)
        self.assertTrue(f.task.memory.last_precise is None or
                        f.task.memory.last_precise['frame_id']!=old)

    def test_bottom_whole_then_partial_during_budgeted_observation_keeps_original_reverse_budget(self):
        b=bottom.BottomScene(self)
        b.begin_reverse();f=b.f
        origin=list(f.task.drive._bottom_reverse_budget['reverse_start_counts'])
        s=CostScene(self,base=f,slices=110)
        self.assertIn('STOP',b.step('whole_far'))
        s.until(lambda:f.task.job is not None and f.task.job['purpose']=='static')
        old=f.task.job['session'].frame['frame_id']
        commands_at=len(f.motor.commands)
        def lose_whole_once(row):
            if len(s.extra)==1:b.mode='partial_lower'
        s.on_extra=lose_whole_once
        s.until(lambda:bool(s.extra))
        s.until(lambda:f.motor.run==1)
        self.assertEqual(f.task.drive._bottom_reverse_budget['reverse_start_counts'],origin)
        self.assertEqual((f.motor.left_speed,f.motor.right_speed),(-15,-15))
        self.assertFalse(f.handoffs)
        self.assertNotIn(old,f.bind_attempts)
        self.assertFalse(any(command.startswith('ARC,') or
            command.startswith(('FOLLOW,','SPEED,')) and
            command.split(',')[1:]!=['-15','-15']
            for unused,command in f.motor.commands[commands_at:]))

    def test_late_feedback_does_not_call_a_complete_bottom_frame_partial(self):
        b=bottom.BottomScene(self)
        b.begin_reverse();f=b.f
        origin=list(f.task.drive._bottom_reverse_budget['reverse_start_counts'])
        s=CostScene(self,base=f,slices=110)
        b.step('whole_far')
        s.until(lambda:f.task.job is not None and f.task.job['purpose']=='static')
        session=f.task.job['session']
        cancelled=f.task.metrics['fine_cancelled']
        def block_one_native_call(row):
            if len(s.extra)==1:f.fine_cost_ms=600
        s.on_extra=block_one_native_call
        def expired_pending_released():
            if not s.extra:return False
            row=s.extra[0]
            pending=(f.task.job or {}).get('_light_pending')
            return (f.clock.now()-row['frame']['capture_timestamp_ms']>
                f.config['tracking']['max_age_ms'] and
                (pending is None or pending['frame_id']!=row['frame']['frame_id']))
        s.until(expired_pending_released)
        row=s.extra[0]
        self.assertGreater(f.clock.now()-row['frame']['capture_timestamp_ms'],
                           f.config['tracking']['max_age_ms'])
        # A delayed encoder bracket may leave this picture too old to classify.
        # Its expiry remains unknown, rather than masquerading as a lost view.
        self.assertNotIn('target',row)
        self.assertEqual(f.task.metrics['fine_cancelled'],cancelled)
        self.assertIs(f.task.job['session'],session)
        self.assertEqual(f.task.drive._bottom_reverse_budget['reverse_start_counts'],origin)
        self.assertTrue(f.task.drive._bottom_reverse_budget['view_recovered'])
        self.assertFalse(f.handoffs)


if __name__=='__main__':unittest.main()
