"""Planning expiry must never dispatch a partial pick or imply infeasibility."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from host.tests import test_standalone_pick as standalone
from host.project_paths import ROOT
import sys
sys.path.insert(0,str(ROOT/'device/k230/pick'))
from planning import PlanningBudget,PlanningTimeout
from kinematics import Geometry
from tools.deploy.k230_pick import build_bundle


class PlanningDeadlineTests(unittest.TestCase):
    def test_exact_deadline_and_tick_wrap(self):
        now=[9998];period=65536
        def delta(a,b):return (a-b+period//2)%period-period//2
        budget=PlanningBudget(10000,lambda:now[0],delta)
        now[0]=19997;budget.check()
        now[0]=19998
        with self.assertRaises(PlanningTimeout) as exc:budget.check()
        self.assertEqual(exc.exception.details,{'limit_ms':10000,'elapsed_ms':10000,'motion_dispatched':False})
        now[0]=65000;budget=PlanningBudget(10000,lambda:now[0],delta)
        now[0]=(65000+10000)%period
        with self.assertRaises(PlanningTimeout):budget.check()

    def test_invalid_budget_is_rejected(self):
        for value in (0,-1,True,None,float('nan'),float('inf')):
            with self.assertRaises(ValueError):PlanningBudget(value)

    def test_expiry_bypasses_orientation_fallback(self):
        cfg,_,_=build_bundle();g=Geometry(cfg['model'],cfg['pick'],-65)
        class Budget:
            def __init__(self,*a):pass
            def check(self):raise PlanningTimeout(10000,10001)
        with patch('planning.PlanningBudget',Budget),patch.object(g,'build_stages') as fallback:
            with self.assertRaises(PlanningTimeout):g.plan([.2677,.028],cfg['ready_control']['goals'],{'status':'estimated','axes_Base':[[1,0,0],[0,1,0]],'table_normal_Base':[0,0,1]})
        fallback.assert_not_called()

    def test_late_result_is_not_returned(self):
        cfg,_,_=build_bundle();g=Geometry(cfg['model'],cfg['pick'],-65);now=[0]
        def build(*args):now[0]=10001;return {'hover':cfg['ready_control']['goals']}
        def budget(limit):return PlanningBudget(limit,lambda:now[0])
        with patch('planning.PlanningBudget',side_effect=budget),patch.object(g,'build_stages',side_effect=build),patch.object(g,'validate_open_fingers'):
            with self.assertRaises(PlanningTimeout):g.plan([.2677,.028],cfg['ready_control']['goals'])

    def test_timeout_logs_distinct_event_and_dispatches_no_stage(self):
        app=standalone.VoiceDispatchTests().app();cfg,_,_=build_bundle()
        class Camera:
            def __init__(self,*a):pass
            def open(self):pass
            def close(self):pass
            def snapshot(self):return SimpleNamespace(save=lambda *a,**k:None)
        class Model:
            def __init__(self,*a):pass
            def plan(self,*a):raise PlanningTimeout(10000,10003)
        with patch.dict(sys.modules,{'cv2':SimpleNamespace(imread=lambda p:object()),'control':SimpleNamespace()}), \
             patch.object(app,'CameraSession',Camera),patch.object(app,'Geometry',Model), \
             patch.object(app,'localize',return_value={'cube_center_Base_mm':[267.7012,27.99938,-58.96532]}), \
             patch.object(app,'arm_snapshot',return_value=[{'position':2048} for _ in range(6)]), \
             patch.object(app,'run_pick_stage_direct') as dispatch,patch.object(app,'log') as log:
            with self.assertRaises(PlanningTimeout):app.run_once(cfg,execute=True)
        dispatch.assert_not_called()
        log.assert_any_call('planning_timeout_no_motion',{'limit_ms':10000,'elapsed_ms':10003,'motion_dispatched':False})

    def test_timeout_returns_to_listener_without_auto_retry(self):
        app=standalone.VoiceDispatchTests().app();calls=[]
        def wait(config,warmup_chunks):
            calls.append(('listen',warmup_chunks))
            if len(calls)>1:raise KeyboardInterrupt()
            return 'pick'
        def run(*a,**kw):calls.append('planning_timeout');raise PlanningTimeout(10000,10001)
        with patch.object(app,'load',return_value={'pick':{'cube_offset_toward_moving_finger_mm':-45},'voice':{}}), \
             patch.object(app,'wait_for_wake',side_effect=wait),patch.object(app,'run_once',side_effect=run),patch.object(app,'log'):
            with self.assertRaises(KeyboardInterrupt):app.run_forever('/fake')
        self.assertEqual(calls,[('listen',32),'planning_timeout',('listen',0)])

if __name__=='__main__':unittest.main()
