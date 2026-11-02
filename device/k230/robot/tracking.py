"""Cooperative persistent task. Callbacks own hardware; no startup motion.

New coarse frames are processed before accepting fine results or stepping the
arm. A fine job owns one original image, epoch and capture time throughout.
Scheduling budgets are soft: native CV, IK and UART calls may still block.
"""
try:
    from ..car.blue_tracker import age_ms
    from ..car.target_memory import TargetMemory
    from ..car.tracking_drive import DriveController
    from ..car.near_runtime import count_delta
    from ..vision.candidate_association import associate_fine_candidate
    from .task import handoff_observation
except ImportError:
    from blue_tracker import age_ms
    from target_memory import TargetMemory
    from tracking_drive import DriveController
    from near_runtime import count_delta
    from candidate_association import associate_fine_candidate
    from robot_task import handoff_observation


class ReobserveRequired(ValueError):
    """The validated target has no usable arm plan from this viewpoint."""


class TrackingTask:
    def __init__(self, capture, track, link, projection, config, clock_ms,
                 begin_arm, moving_factory, static_factory, evidence=None,
                 emit=None, reset_tracker=None):
        self.capture, self.track, self.link = capture, track, link
        self.config, self.clock, self.begin_arm = config, clock_ms, begin_arm
        self.factories = {'moving': moving_factory, 'static': static_factory}
        self.evidence = evidence if evidence is not None else []
        self.emit = emit or (lambda event, details: None)
        self.reset_tracker = reset_tracker or (lambda: None)
        self.arm_context = None
        self.memory, self.drive = TargetMemory(projection, config), DriveController(projection, config)
        timing = config.get('tracking_task', {})
        self.period = timing.get('period_ms', 100)
        self.stationary_period = timing.get('stationary_period_ms', 250)
        self.stationary_status_period = timing.get('stationary_status_period_ms', 500)
        synced_at = self.clock()
        self.state = self.link.sync()
        self.state['_k230_sent_ms'], self.state['_k230_received_ms'] = synced_at, self.clock()
        self._last_motor_reply = self.state['_k230_received_ms']
        self.coarse = self.target = self.decision = None
        self.arm = self.job = self.complete = None
        self.planning = None
        self.last_parking = None
        self._parking_version = None
        self._last_capture = self._frame_motor = None
        self._fresh_before_arm = False
        self._force_observe = False
        self._arm_motor_reference = None
        self._recovery_seen = False
        self._missing = 0
        self.fault = None
        self.status = 'running'
        self.timeline = []
        self._last_drive_event = self._last_target_event = None
        self.metrics = dict(observations=0, max_observation_gap_ms=0, max_step_ms=0,
            fine_started=0, fine_cancelled=0, arm_plans=0, arm_attempts=0, recoveries=0,
            planning_steps=0, plans_cancelled=0, max_planning_step_ms=0,
            target_changes=0, obsolete_frames_released=0, status_queries=0,
            first_motion_request=None)

    def _record(self, event, details):
        # Keep transitions in RAM during motion. They join the existing final
        # run record; no per-frame SD writes or JPEG copies are introduced.
        self.timeline.append(dict(at_ms=self.clock(), event=event, details=details))
        self.timeline[:] = self.timeline[-64:]
        self.emit(event, details)

    def _command(self, command):
        sent = self.clock()
        state = self.link.expect(command)
        if (self.metrics['first_motion_request'] is None and
                command.split(',')[0] in ('FOLLOW', 'SPEED', 'ARC') and
                any(value != '0' for value in command.split(',')[1:])):
            self.metrics['first_motion_request'] = dict(command=command, sent_at_ms=sent,
                reply_at_ms=self.clock())
        state['_k230_sent_ms'], state['_k230_received_ms'] = sent, self.clock()
        self.state = state
        self._last_motor_reply = state['_k230_received_ms']
        if command == 'STATUS':
            self.metrics['status_queries'] += 1
            # The first control decision must still see an unowned running
            # motor as initially busy. Thereafter latch every real sample,
            # including excursions between camera frames; Memory stays frame-owned.
            if self.drive._motor is not None:
                self.drive._observe_motor(state, self._last_motor_reply)
        return state

    def _moving_observation(self):
        return (self.state.get('run') or
                (self.job is not None and self.job['purpose'] == 'moving') or
                (self.complete is not None and self.complete['purpose'] == 'moving'))

    def _observation_period(self):
        if self._moving_observation() or self.arm is not None:
            return self.period
        if (self.planning is not None or
                (self.job is not None and self.job['purpose'] == 'static') or
                (self.complete is not None and self.complete['purpose'] == 'static')):
            return self.stationary_period
        return self.period

    def _status_due(self, at_observation=False):
        # Moving feedback and initial stop confirmation keep their original
        # cadence. Once stopped, STATUS has its own slower deadline, independent
        # of how often the camera observes; reusing a reply never renews its age.
        if self._moving_observation():
            return at_observation
        return ((at_observation and self.drive.stopped_at_ms is None) or
                age_ms(self.clock(), self._last_motor_reply) >= self.stationary_status_period)

    def idle_delay_ms(self):
        """Stopped background calculation can use its next slice immediately."""
        if (self.fault is None and self.arm is None and not self.state.get('run') and
                (self.job is not None or self.planning is not None) and
                (self.target or {}).get('movement_state') != 'suspected'):
            return 0
        return 5

    def _fault(self, stage, error):
        reason = '%s: %s' % (stage, error)
        changed=self.fault != reason
        self.fault = reason
        # A lost ACK may leave state.run stale even after the MCU began motion.
        # Stop before logging or generator cleanup, either of which may fail.
        try:
            self._command('STOP')
        except Exception:
            pass  # Existing MCU communication lease still applies.
        for cancel in (self._cancel_job,self._cancel_plan):
            try:
                cancel('execution_fault')
            except Exception as exc:
                self.metrics['cleanup_error']=str(exc)
        if changed:
            try:self.emit('tracking_fault_wait', {'reason': reason, 'task_terminal': False})
            except Exception:pass

    def _save_job(self, reason=None):
        if self.job is None:
            return
        job = self.job
        try:
            item = job['session'].take_evidence('tracking_%d_%s' % (
                job['number'], job['purpose']))
        except Exception as exc:
            self.emit('vision_archive_error', {'error':str(exc)})
            return
        if item:
            item['frame'] = dict(item['frame'], target_version=job['target_version'])
            item['frame']['tracking_purpose'] = job['purpose']
            # Keep the actual input with the image; a derived search ROI cannot
            # reconstruct the original box or which colour candidate seeded it.
            coarse = job['paired_coarse']
            item['frame']['paired_coarse'] = {key: coarse[key] for key in (
                'frame_id', 'capture_timestamp_ms', 'status', 'bbox_px',
                'center_pixel', 'confirmations', 'ground_contact_proxy_Base_mm',
                'visual_evidence', 'visual_candidates') if key in coarse}
            item['frame']['localization_bbox_px'] = job['localization_bbox_px']
            if reason:
                item['frame']['discard_reason'] = reason
            self._release_evidence(job['target_version'], job['purpose'])
            self.evidence.append(item)

    def _release_evidence(self, version, replaced_purpose=None):
        """Keep the last moving/static diagnostic images, across target loss.

        They never grant control authority. Replacing the same purpose still
        bounds RAM to two frames, without SD writes or a stop during motion.
        """
        retained = [item for item in self.evidence
            if replaced_purpose is None or item['frame'].get('tracking_purpose') != replaced_purpose]
        self.metrics['obsolete_frames_released'] += len(self.evidence)-len(retained)
        self.evidence[:] = retained

    def _cancel_job(self, reason):
        try:
            if self.job is not None:
                if reason in ('execution_fault', 'task_closed'):
                    self._save_job(reason)
                cancel = getattr(self.job['session'], 'cancel', None)
                if cancel:cancel()
                self.metrics['fine_cancelled'] += 1
        finally:
            self.job = self.complete = None
            self.drive.cancel_fine()

    def _wheel_delta(self, previous, current):
        if previous is None:
            return ()
        scale = self.config['mm_per_count']
        return [(count_delta(previous['encoder_a'], current['encoder_a'])*scale,
                 -count_delta(previous['encoder_b'], current['encoder_b'])*scale)]

    def _check_chassis_reference(self):
        if ((self.arm is None and self.planning is None) or
                self._arm_motor_reference is None):
            return
        if not self.state.get('run') and not any(
                abs(count_delta(self._arm_motor_reference[key], self.state[key]))*
                self.config['mm_per_count'] > .1 for key in ('encoder_a', 'encoder_b')):
            return
        self.memory.invalidate('chassis_moved_during_grasp')
        self.target = self.memory.snapshot()
        self.last_parking = None
        self._cancel_plan('chassis_moved_during_grasp')
        self.drive.restart_observation()
        if self.state.get('run'):
            self._command('STOP')

    def _cancel_plan(self, reason):
        if self.planning is None:
            return
        plan, self.planning = self.planning, None
        close = getattr(plan['iterator'], 'close', None)
        if close:
            close()
        self.metrics['plans_cancelled'] += 1
        self.emit('tracking_plan_cancelled', {'reason':reason,
            'target_version':plan['target_version']})

    def _advance_plan(self):
        if self.planning is None or self.target.get('movement_state') == 'suspected':
            return
        plan = self.planning
        started = self.clock()
        try:
            event = next(plan['iterator'])
            self.metrics['planning_steps'] += 1
            if event.get('target_version') != plan['target_version']:
                raise ValueError('Planning changed its original target version')
            if event.get('stage') == 'complete':
                engine = event['execution']
                if (event.get('frame_id') != plan['frame_id'] or
                        engine.snapshot()['target_version'] != plan['target_version']):
                    engine.close()
                    raise ValueError('Planning changed its original observation')
                self.arm = engine
                close = getattr(plan['iterator'], 'close', None)
                if close:
                    close()
                self.planning = None
                self._fresh_before_arm = True
                self._record('tracking_plan_ready', dict(target_version=plan['target_version'],
                    frame_id=plan['frame_id'], elapsed_ms=age_ms(self.clock(), plan['started_at_ms'])))
        except ReobserveRequired as exc:
            self._cancel_plan('scene_has_no_arm_plan')
            self.last_parking = None
            self.emit('tracking_reobserve', {'reason':str(exc), 'stage':'arm_plan'})
            self.drive.restart_observation(change_view=True)
            self._force_observe = True
        except Exception as exc:
            self._fault('arm_plan', exc)
        finally:
            self.metrics['max_planning_step_ms'] = max(self.metrics['max_planning_step_ms'],
                age_ms(self.clock(), started))

    def _observe(self, force_status=False):
        image, frame = self.capture()
        now = frame['capture_timestamp_ms']
        if self._last_capture is not None:
            self.metrics['max_observation_gap_ms'] = max(
                self.metrics['max_observation_gap_ms'], age_ms(now, self._last_capture))
        self._last_capture = now
        self.metrics['observations'] += 1
        if self.fault is None and (force_status or self._status_due(at_observation=True)):
            try:
                self._command('STATUS')
            except Exception as exc:
                self._fault('motor_status', exc)
        self.coarse = self.track(image, frame)
        arm_state = self.arm.snapshot() if self.arm is not None else None
        if arm_state is not None and self.arm_context is not None:
            arm_state = self.arm_context(arm_state)
        deltas = self._wheel_delta(self._frame_motor, self.state)
        self._check_chassis_reference()
        self.target = self.memory.update(self.coarse, deltas, arm_state,
                                         motion_state=self.state)
        if self._parking_version != self.memory.version:
            self.last_parking = None
        if self.planning is not None and (
                self.planning['target_version'] != self.memory.version or
                not self.target.get('target_valid') or
                (self.target.get('visible') and self.target.get('associated') and
                 self.target.get('movement_state') != 'suspected' and
                 self.target.get('needs_precise'))):
            self._cancel_plan('observation_changed_during_planning')
            self.last_parking = None
            self.drive.restart_observation()
        self._release_evidence(self.memory.version)
        self._frame_motor = dict(self.state)
        if self.target['confirmed_moved']:
            self.metrics['target_changes'] += 1
        change = (self.memory.version, self.target['event'], self.target['movement_state'])
        if change != self._last_target_event:
            self._last_target_event = change
            self._record('tracking_target', dict(target_version=self.memory.version,
                event=self.target['event'], movement_state=self.target['movement_state'],
                reason=self.target.get('reason'), residual_px=self.target.get('residual_px'),
                bbox_px=self.coarse.get('bbox_px'),
                visual_evidence=self.coarse.get('visual_evidence')))
        if self.job is not None and self.job['target_version'] != self.memory.version:
            self._cancel_job('target_version_changed')
        if self.complete is not None and self.complete['target_version'] != self.memory.version:
            self.complete = None
            self.drive.cancel_fine()
        evidence = self.coarse.get('visual_evidence') or {}
        if (self.target['event'] in ('moved', 'acquired', 'reappeared') and
                self.target['associated'] and evidence.get('associated') is False):
            # Memory has confirmed a unique jump; the next native frame can
            # establish a new color association instead of following old blue.
            self.reset_tracker()
        self._missing = (self._missing+1 if not (
            self.target['visible'] and self.target['associated'] or
            self.target.get('precise_candidate')) else 0)
        unsent_hover = self.arm is not None and self.arm.snapshot().get('pending_stage') == 'hover'
        if (self._missing >= self.config['tracking'].get('confirmation_frames', 3) and
                (self.planning is not None or unsent_hover)):
            # A brief gap preserves work. When the existing search policy is
            # due, drop only unsent work so its ownership cannot block searching.
            self._cancel_plan('target_unseen_before_hover')
            if unsent_hover:
                self.arm.close()
                self.arm = None
            self._arm_motor_reference = self.last_parking = None
            self.drive.restart_observation()
        self._fresh_before_arm = False
        return image, frame

    def _start_job(self, purpose, image, frame):
        fine_config = dict(self.config)
        fine_config['near_parking'] = dict(self.config['near_parking'],
            fine_budget_ms=self.config.get('tracking_task', {}).get('fine_budget_ms', 20))
        session = self.factories[purpose](fine_config, self.clock)
        self.metrics['fine_started'] += 1
        source=self.coarse.get('visual_evidence') or {}
        bbox=(list(source['bbox_px']) if source.get('associated') is True and
              source.get('ambiguous') is False and source.get('bbox_px') else None)
        self.job = {'session': session, 'purpose': purpose,
            'target_version': self.memory.version, 'paired_coarse': dict(self.coarse),
            'localization_bbox_px':bbox,
            'request_id': self.decision.get('fine_request_id'),
            'number': self.metrics['fine_started']}
        self._record('tracking_fine_started', dict(purpose=purpose,
            frame_id=frame['frame_id'], target_version=self.memory.version,
            capture_timestamp_ms=frame['capture_timestamp_ms']))
        try:
            session.start(image, frame, bbox)
        except Exception as exc:
            self._fault('fine_start',exc)

    def _advance_job(self):
        if self.job is None:
            return
        job = self.job
        try:
            job['session'].advance()
        except Exception as exc:
            self._fault('fine_compute',exc)
            return
        if not job['session'].phase:
            observation = job['session'].latest
            if observation.get('status') == 'localized' or observation.get('range_candidate'):
                association = self._match_fine(job['paired_coarse'], observation)
                # Retain the raw visual solve with its association audit. Only
                # the filtered capability below may reach driving or Memory.
                job['session'].latest = dict(observation, target_association=association)
                job['session'].latest_ready_ms = self.clock()
                observation = job['session'].latest
                if association['status'] != 'matched':
                    observation = dict(observation, status='no_complete_face',
                        reason='Fine result is not associated with the selected target')
                    observation.pop('cube_center_Base_mm', None)
                    observation.pop('range_candidate', None)
            self.complete = dict(job, observation=observation,
                result_ready_ms=job['session'].latest_ready_ms)
            self.complete.pop('session')
            self._record('tracking_fine_completed', dict(purpose=job['purpose'],
                frame_id=job['session'].latest.get('frame_id'),
                status=observation.get('status'),
                target_association=(observation.get('target_association') or {}).get('status'),
                reason=observation.get('reason'),search=observation.get('search'),
                request_id=job['request_id'],
                elapsed_ms=age_ms(job['session'].latest_ready_ms,
                    job['session'].latest.get('capture_timestamp_ms', job['session'].latest_ready_ms))))
            self._save_job()
            self.job = None

    def _match_fine(self, coarse, observation):
        return associate_fine_candidate(observation, coarse)

    def _step_arm(self):
        if self.arm is None or self._fresh_before_arm:
            return
        before = self.arm.snapshot()
        if before['target_version'] == self.memory.version and before.get('target_valid', True):
            if before.get('pending_stage') == 'hover':
                if (not self.target.get('associated') or not self.target.get('visible') or
                        self.target.get('movement_state') == 'suspected'):
                    # Unknown is a pause, not evidence that the plan moved.
                    # A later confirmed version change still cancels it below.
                    return
            if before.get('pending_stage') == 'hover' and self.target.get('needs_precise'):
                # No servo goal has been sent by this engine. Drop the stale
                # plan without a physical recovery action, then obtain a new
                # static result; unknown vision alone must not open the jaw.
                self.arm.close()
                self.arm = None
                self._arm_motor_reference = None
                self.drive.restart_observation()
                self._force_observe = True
                return
            if (self.target.get('movement_state') == 'suspected' and
                    (before.get('pending_stage') is not None or before['phase'] == 'descend_lead')):
                return
        dispatching_first = (before.get('pending_stage') == 'hover' and
            before['target_version'] == self.memory.version and before.get('target_valid', True))
        allow_success = self.target.get('movement_state') != 'suspected'
        if before['target_version'] != self.memory.version or not before.get('target_valid', True):
            state = self.arm.step(target_version=self.memory.version, allow_success=allow_success)
        elif before.get('next_poll_in_ms', 0) <= 0:
            state = self.arm.step(target_version=self.memory.version, allow_success=allow_success)
        else:
            state = before
        if dispatching_first and state.get('pending_stage') != 'hover':
            self.metrics['arm_attempts'] += 1
            self._record('tracking_first_hover', dict(target_version=self.memory.version,
                static_frame_id=(self.last_parking or {}).get('verification_observation', {}).get('frame_id'),
                photo_to_hover_ms=age_ms(self.clock(), (self.last_parking or {}).get(
                    'verification_observation', {}).get('capture_timestamp_ms', self.clock()))))
        if state['status'] in ('recovering', 'reobserve_ready') and not self._recovery_seen:
            self._recovery_seen = True
            self.metrics['recoveries'] += 1
            self.memory.invalidate(state.get('recovery_reason') or 'arm_recovery')
            self.last_parking = None
            self._release_evidence(self.memory.version)
            self._cancel_job('arm_recovery')
            self.drive.restart_observation()
        if state['status'] == 'fault':
            self._fault('arm_execution', state.get('error'))
        elif state['status'] == 'succeeded':
            self.status = 'succeeded'
        elif state.get('can_reobserve'):
            self.arm.close()
            self.arm = None
            self._arm_motor_reference = None
            self._recovery_seen = False
            self.drive.restart_observation()
            self._force_observe = True

    def _advance_background(self):
        """Spend a short time slice, not one sleep for every cheap yield.

        Hundreds of algebraic progress events should not each cost5ms of idle
        time. Stop at the next observation deadline or the soft work budget.
        """
        started = self.clock()
        budget = self.config.get('tracking_task', {}).get('fine_budget_ms', 20)
        while self.fault is None and self.arm is None:
            if (age_ms(self.clock(), started) >= budget or
                    (self._last_capture is not None and
                     age_ms(self.clock(), self._last_capture) >= self._observation_period())):
                return
            if self.planning is not None:
                if self.target.get('movement_state') == 'suspected':
                    return
                self._advance_plan()
            elif self.job is not None:
                self._advance_job()
                if self.complete is not None:
                    return
            else:
                return

    def _step(self):
        """Advance one cooperative slice; recoverable scenes never end the task."""
        if self.status == 'succeeded':
            return self.snapshot()
        started = self.clock()
        observed = (self._last_capture is None or self._fresh_before_arm or self._force_observe or
                    (self.complete is not None and
                     not (self.decision or {}).get('precise_deferred')) or
                    age_ms(started, self._last_capture) >= self._observation_period())
        image = frame = None
        if observed:
            # These two handoffs retain a current chassis sample as well as a
            # current coarse frame; ordinary stationary observations do not.
            image, frame = self._observe(force_status=
                self._fresh_before_arm or self.complete is not None)
            self._force_observe = False
            if self.fault is None:
                event = dict(self.target)
                # An unknown center loss is first observed, not called an exit.
                # Once stopped, ordinary confirmation frames may start looking.
                if (self.arm is None and self.planning is None and not self.state['run'] and self.job is None and
                        self.complete is None and (not event['anchor_valid'] or
                        self.drive.phase in ('settling', 'observing', 'observe')) and self._missing >=
                        self.config['tracking'].get('confirmation_frames', 3)):
                    event['search'] = True
                eligible = self.complete if (self.target.get('target_valid') or
                                              self.target.get('precise_candidate')) else None
                outstanding_request=(self.drive._fine_request or {}).get('request_id')
                self.decision = self.drive.update(self.coarse, self.state, self.clock(),
                    precise=eligible, target_version=self.memory.version,
                    fine_pending=self.job is not None, target_event=event,
                    arm_active=self.arm is not None or self.planning is not None)
                if self.decision.get('reset_tracker'):
                    self.reset_tracker()
                if self.complete is not None and not self.decision.get('precise_deferred'):
                    self._record('tracking_fine_consumed',dict(
                        purpose=self.complete['purpose'],
                        frame_id=self.complete['observation'].get('frame_id'),
                        offered_request_id=self.complete.get('request_id'),
                        outstanding_request_id=outstanding_request,
                        consumed=self.decision.get('precise_consumed'),
                        anchor_present=self.drive.anchor is not None,
                        phase=self.decision.get('phase')))
                drive_event = (self.decision.get('phase'), self.decision.get('status'),
                    (self.decision.get('command') or '').split(',')[0])
                if drive_event != self._last_drive_event:
                    self._last_drive_event = drive_event
                    self._record('tracking_drive', dict(phase=drive_event[0],
                        status=drive_event[1], command=self.decision.get('command'),
                        target_version=self.memory.version,
                        encoder_a=self.state.get('encoder_a'), encoder_b=self.state.get('encoder_b')))
                if self.complete is not None and self.decision.get('precise_consumed'):
                    done = self.complete
                    if done['observation'].get('target_association', {}).get('status') == 'matched':
                        bound=self.memory.bind_precise(done['observation'], done['paired_coarse'],
                            done['target_version'], self.drive.odometry.since(
                                done['observation'].get('capture_timestamp_ms', self.clock()), self.clock()))
                        if not bound:
                            self.last_parking=None
                            self.drive.restart_observation(discard_reference=True)
                            self.decision.update(allow_arm=False,parking=None,phase='settling',
                                command='STOP' if self.state.get('run') else None)
                            self._force_observe=True
                            self._record('tracking_fine_binding_rejected',dict(
                                frame_id=done['observation'].get('frame_id')))
                if not self.decision.get('precise_deferred'):
                    self.complete = None
                if self.decision.get('command'):
                    try:
                        self._command(self.decision['command'])
                    except Exception as exc:
                        self._fault('motor_command', exc)
                if self.arm is None and self.planning is None and self.fault is None:
                    if self.decision.get('allow_arm'):
                        try:
                            observation = handoff_observation(self.decision['parking'])
                            self.last_parking = self.decision['parking']
                            self._parking_version = self.memory.version
                            self._record('tracking_plan_started', dict(target_version=self.memory.version,
                                frame_id=observation['frame_id']))
                            planning_started = self.clock()
                            candidate = self.begin_arm(observation, self.memory.version)
                            self._arm_motor_reference = dict(self.drive._parking_reference)
                            self.metrics['arm_plans'] += 1
                            if hasattr(candidate, 'snapshot'):
                                # Compatibility with an inert synchronous
                                # factory; also require a new frame after it.
                                self.arm = candidate
                                self._fresh_before_arm = True
                            else:
                                self.planning = {'iterator':iter(candidate),
                                    'target_version':self.memory.version,
                                    'frame_id':observation['frame_id'], 'started_at_ms':planning_started}
                        except ReobserveRequired as exc:
                            self.last_parking = None
                            self.emit('tracking_reobserve', {'reason': str(exc), 'stage': 'arm_plan'})
                            self.drive.restart_observation(change_view=True)
                        except Exception as exc:
                            self._fault('arm_plan', exc)
                    elif (self.decision.get('request_fine') and self.job is None and
                            (self.target.get('target_valid') or
                             self.decision['request_fine']=='static' and
                             self.target.get('precise_candidate'))):
                        self._start_job(self.decision['request_fine'], image, frame)
        elif self.fault is None and self._status_due():
            try:
                self._command('STATUS')
                self._check_chassis_reference()
            except Exception as exc:
                self._fault('motor_status', exc)
        if self.fault is None:
            self._step_arm()
            self._advance_background()
        self.metrics['max_step_ms'] = max(self.metrics['max_step_ms'], age_ms(self.clock(), started))
        return self.snapshot()

    def step(self):
        started = self.clock()
        try:
            return self._step()
        except Exception as exc:
            self._fault('tracking_io', exc)
            return self.snapshot()
        finally:
            self.metrics['max_step_ms'] = max(self.metrics['max_step_ms'], age_ms(self.clock(), started))

    def snapshot(self):
        state = self.arm.snapshot() if self.arm is not None else None
        return {'schema': 'aic.robot.tracking.v1', 'status': self.status,
            'target_version': self.memory.version, 'target': self.memory.snapshot(),
            'phase': state['phase'] if state else ('planning' if self.planning is not None else
                      (self.decision or {}).get('phase', 'observe')),
            'planning_pending':self.planning is not None, 'fine_pending':self.job is not None,
            'arm_started': self.metrics['arm_attempts'] > 0, 'arm_state': state,
            'parking': self.last_parking or (self.decision or {}).get('parking'),
            'grasp_confirmed': (state or {}).get('grasp_confirmed'),
            'grip_confirmation': (state or {}).get('grip_confirmation'),
            'physical_grasp_confirmed': False, 'fault_wait': self.fault,
            'timeline': list(self.timeline),
            'motion_wait_reason': (self.decision or {}).get('motion_wait_reason'),
            'arc_timeouts': self.drive.arc_timeouts, 'last_arc_end': self.drive.last_arc_end,
            'corrections': self.drive.corrections, 'metrics': dict(self.metrics)}

    def close(self):
        try:
            self._cancel_job('task_closed')
        finally:
            try:
                self._cancel_plan('task_closed')
            finally:
                try:
                    self._command('STOP')
                finally:
                    if self.arm is not None:
                        self.arm.close()


def run_tracking(*args, sleep_ms=None, **kwargs):
    task = TrackingTask(*args, **kwargs)
    try:
        while task.status != 'succeeded':
            task.step()
            delay = task.idle_delay_ms()
            if sleep_ms and delay:
                sleep_ms(delay)
        return task.snapshot()
    finally:
        task.close()
