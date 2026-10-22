"""Interactive recorder uses separate registrations; all board I/O mocked."""
import ast
import contextlib
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from tools.diagnostics import enroll_k230_wake_word as enroll
from host.project_paths import ROOT


class EnrollmentTests(unittest.TestCase):
    def run_enrollment(self,bad_result=False,listen=False):
        source=json.loads((ROOT/'configs/voice/wake_word.json').read_text())
        source['commands']['ready']['enrollment_file']=None
        original=json.loads(json.dumps(source))
        calls=[];stops=[];closed=[]
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);cfg=root/'configs/voice/wake_word.json'
            cfg.parent.mkdir(parents=True);cfg.write_text(json.dumps(source,ensure_ascii=False))
            audio=root/'device/k230/audio/wake_word.py';audio.parent.mkdir(parents=True);audio.write_text('# test')
            def execute(link,source,call,timeout,phrase):
                self.assertEqual(phrase,'回准备位');calls.append(call)
                args=ast.literal_eval('('+call.split('(',1)[1][:-1]+',)')
                if call.startswith('capture_take('):
                    # Reject the first take once; only this take should repeat.
                    return ['VOICE_TAKE '+json.dumps({'ok':len(calls)!=1,'reason':'noise','number':args[1]})]
                if call.startswith('finalize('):
                    self.assertEqual(args[1:],('ready','回准备位'))
                    return ['VOICE_ENROLLED '+json.dumps({'path':args[0]+'/keyword.json',
                            'keyword':{'kw_id':'wrong' if bad_result else 'ready'}})]
                raise AssertionError('Unexpected board function: '+call)
            link=SimpleNamespace(port=SimpleNamespace(close=lambda:closed.append(True)),command=lambda c:stops.append(c))
            with patch.object(enroll,'ROOT',root),patch.object(enroll,'CameraLink',return_value=link) as connect,\
                 patch.object(enroll,'execute',side_effect=execute),patch('builtins.input',return_value=''),\
                 contextlib.redirect_stdout(io.StringIO()):
                code=enroll.main(['--command','ready']+(['--listen'] if listen else []))
            saved=json.loads(cfg.read_text())
        return code,original,saved,calls,stops,closed,connect.call_count

    def test_three_takes_retry_and_preserve_pick_registration(self):
        code,original,saved,calls,stops,closed,count=self.run_enrollment()
        self.assertEqual(code,0);self.assertEqual(count,1);self.assertEqual(len(calls),5)
        numbers=[ast.literal_eval('('+c.split('(',1)[1][:-1]+',)')[1] for c in calls[:-1]]
        self.assertEqual(numbers,[1,1,2,3])
        command=saved['commands']['ready'];self.assertTrue(command['enrollment_file'].startswith('/sdcard/voice/ready_'))
        saved['commands']['ready']=original['commands']['ready']
        self.assertEqual(saved,original);self.assertEqual(stops,[]);self.assertEqual(closed,[True])

    def test_wrong_result_never_replaces_either_registration(self):
        code,original,saved,calls,stops,closed,count=self.run_enrollment(bad_result=True)
        self.assertEqual(code,1);self.assertEqual(saved,original);self.assertEqual(stops,[0x06])

    def test_listen_before_enrollment_does_not_even_connect(self):
        code,original,saved,calls,stops,closed,count=self.run_enrollment(listen=True)
        self.assertEqual(code,1);self.assertEqual(count,0);self.assertEqual(calls,[])

    def test_prompt_names_selected_phrase(self):
        link=SimpleNamespace(execute=lambda source,timeout:iter([b'VOICE_READY 1',b'VOICE_SPEECH']))
        with contextlib.redirect_stdout(io.StringIO()) as output:
            enroll.execute(link,'# test','capture_take("/fake",1)',35,'回准备位')
        self.assertIn('现在请说：回准备位',output.getvalue());self.assertNotIn('思达思达',output.getvalue())

if __name__=='__main__':unittest.main()
