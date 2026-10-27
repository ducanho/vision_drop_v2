import ast
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from core.settings import ROOT,owned


class IndependenceTests(unittest.TestCase):
    def test_application_imports_have_no_legacy_path(self):
        banned=['nx100_camera_snapshot_move','nx100_web_control','nx100_variables_web',
                'nx100_yolo_auto_tracking','nx100-remote-control-master','yolo_hikrobot_robot_coordinates']
        paths=list(ROOT.glob('*.py'))
        for folder in ['core','robot','vision','workflow']:
            paths+=list((ROOT/folder).glob('*.py'))
        for path in paths:
            text=path.read_text(encoding='utf-8')
            tree=ast.parse(text)
            for name in banned:
                self.assertNotIn(name,text,str(path))
            for node in ast.walk(tree):
                if isinstance(node,ast.Attribute):
                    self.assertNotEqual(ast.unparse(node),'sys.path',str(path))
            self.assertNotIn('C:\\Data',text,str(path))
            self.assertFalse(path.is_symlink())
        config=json.dumps(json.loads(owned('config/config.json').read_text()))
        self.assertNotIn('C:',config)

    def test_owned_path_cannot_escape(self):
        with self.assertRaises(ValueError):
            owned('../outside.pt')

    def test_copied_assets_integrity(self):
        hashes=json.loads(owned('docs/asset_hashes.json').read_text())
        for relative,expected in hashes.items():
            self.assertEqual(hashlib.sha256(owned(relative).read_bytes()).hexdigest(),expected)

    def test_relocated_mock_cycle_without_access_to_original_project(self):
        # Fresh subprocess imports relocated app only; audit rejects access to old workspace.
        # System Python/third-party site-packages are permitted as installed dependencies.
        with tempfile.TemporaryDirectory(prefix='nx100-relocated-') as temporary:
            root=Path(temporary)
            for name in ['app.py','core','config','robot','vision','workflow','calibration','models','templates','static']:
                src=ROOT/name
                if src.is_dir():
                    shutil.copytree(src,root/name,ignore=shutil.ignore_patterns('__pycache__'))
                else:
                    shutil.copy2(src,root/name)
            code='''
import sys,os,time
from pathlib import Path
original=Path(os.environ['NX100_ORIGINAL']).resolve()
def guard(event,args):
 if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)):
  p=Path(os.fsdecode(args[0])).resolve()
  if p.is_relative_to(original) and not p.is_relative_to(original/'.venv'):
   raise RuntimeError('Original project access rejected: '+str(p))
sys.addaudithook(guard)
from app import create_app
app=create_app(mock=True)
c=app.state.cycle
c.cfg['workflow']['repeat_until_empty']=False
assert not c.robot.client.commands
c.connect()
deadline=time.monotonic()+4
while not c.vision.status()['connected'] and time.monotonic()<deadline: time.sleep(.02)
c.start('relocated-start-01',c.journal.state['revision'])
deadline=time.monotonic()+10
while c.busy and time.monotonic()<deadline: time.sleep(.02)
assert c.journal.state['state']=='COMPLETE',c.journal.snapshot()
assert c.stack.snapshot()['stacked_count']==1
for name,module in list(sys.modules.items()):
 if name.split('.')[0] in ('app','core','robot','vision','workflow') and getattr(module,'__file__',None):
  assert Path(module.__file__).resolve().is_relative_to(Path.cwd())
c.close()
print('RELOCATED_MOCK_COMPLETE')
'''
            env={**os.environ,'PYTHONPATH':'','NX100_ORIGINAL':str(ROOT),'PYTHONDONTWRITEBYTECODE':'1','NX100_HMI_TOKEN':''}
            result=subprocess.run([sys.executable,'-c',code],cwd=root,env=env,text=True,capture_output=True,timeout=25)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertIn('RELOCATED_MOCK_COMPLETE',result.stdout)
