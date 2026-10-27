import io
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch

from app import create_app,main
from core.launcher import APPLICATION_ID,existing_real_hmi,hmi_url,open_edge,open_when_ready
from core.settings import load_config


class LauncherTests(unittest.TestCase):
    def test_factory_defaults_to_real_without_connecting_hardware(self):
        with tempfile.TemporaryDirectory() as tmp:
            app=create_app(journal_path=Path(tmp)/'execution.json')
            c=app.state.cycle
            try:
                self.assertEqual(c.snapshot()['mode'],'REAL')
                self.assertFalse(c.robot.connected)
                self.assertIsNone(c.thread)
            finally:
                c.close()

    def test_main_defaults_to_real_and_schedules_edge_after_startup(self):
        cfg=load_config()
        server=Mock()
        with patch('sys.argv',['app.py']),patch('app.load_config',return_value=cfg),\
             patch('app.existing_real_hmi',return_value=False),\
             patch('app.create_app') as factory,patch('uvicorn.Config') as config,\
             patch('uvicorn.Server',return_value=server),patch('app.threading.Thread') as thread,\
             patch('app.RotatingFileHandler'),patch('app.logging.basicConfig'):
            main()
        factory.assert_called_once_with(mock=False,config=cfg)
        self.assertEqual(config.call_args.kwargs['port'],8011)
        thread.assert_called_once_with(target=open_when_ready,args=(server,'http://127.0.0.1:8011'),daemon=True,name='open-edge')
        thread.return_value.start.assert_called_once()
        server.run.assert_called_once()

    def test_second_launch_opens_existing_real_hmi_without_second_server(self):
        with patch('sys.argv',['app.py']),patch('app.existing_real_hmi',return_value=True),\
             patch('app.open_edge') as edge,patch('app.create_app') as factory,\
             patch('app.RotatingFileHandler'),patch('app.logging.basicConfig'):
            main()
        edge.assert_called_once_with('http://127.0.0.1:8011')
        factory.assert_not_called()

    def test_edge_launch_uses_argument_list(self):
        with patch('core.launcher.shutil.which',return_value='C:/Edge/msedge.exe'),\
             patch('core.launcher.subprocess.Popen') as launch:
            open_edge('http://127.0.0.1:8011')
        launch.assert_called_once_with(['C:/Edge/msedge.exe','--new-tab','http://127.0.0.1:8011'])

    def test_edge_waits_for_server_ready_and_does_not_open_failed_server(self):
        server=SimpleNamespace(started=False,should_exit=False)
        def ready(_):
            server.started=True
        with patch('core.launcher.time.sleep',side_effect=ready) as sleep,patch('core.launcher.open_edge') as edge:
            open_when_ready(server,'http://127.0.0.1:8011')
        sleep.assert_called_once()
        edge.assert_called_once()
        with patch('core.launcher.open_edge') as edge:
            open_when_ready(SimpleNamespace(started=False,should_exit=True),'http://127.0.0.1:8011')
        edge.assert_not_called()

    def test_existing_server_must_match_application_and_real_mode(self):
        for state,expected in [({'application_id':APPLICATION_ID,'mode':'REAL'},True),
                               ({'application_id':APPLICATION_ID,'mode':'MOCK'},False),
                               ({'application_id':'other','mode':'REAL'},False)]:
            with patch('core.launcher.urllib.request.urlopen',return_value=io.BytesIO(json.dumps(state).encode())):
                self.assertEqual(existing_real_hmi('http://127.0.0.1:8011'),expected)
        self.assertEqual(hmi_url('0.0.0.0',8011),'http://127.0.0.1:8011')
