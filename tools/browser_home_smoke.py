"""Headless HOME button checks against a private MOCK server; block robot TCP."""
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import urllib.request

from playwright.sync_api import sync_playwright, expect
from core.settings import ROOT, owned


def main():
    report=owned('logs/reports/home_browser')
    report.mkdir(parents=True,exist_ok=True)
    with socket.socket() as listener:
        listener.bind(('127.0.0.1',0))
        port=listener.getsockname()[1]
    with tempfile.TemporaryDirectory(prefix='nx100-home-mock-') as temporary:
        journal=Path(temporary)/'execution.json'
        code='''
import sys,threading,time,socket
from pathlib import Path
from unittest.mock import patch
patch('socket.create_connection',side_effect=AssertionError('Robot TCP disabled')).start()
from app import create_app
import uvicorn
app=create_app(mock=True,journal_path=Path(sys.argv[1]))
server=uvicorn.Server(uvicorn.Config(app,host='127.0.0.1',port=int(sys.argv[2]),log_level='error'))
def stop():
 while not Path(sys.argv[1]).with_suffix('.stop').exists(): time.sleep(.1)
 server.should_exit=True
threading.Thread(target=stop,daemon=True).start()
server.run()
'''
        with (report/'server.log').open('w',encoding='utf-8') as log:
            process=subprocess.Popen([sys.executable,'-c',code,str(journal),str(port)],cwd=ROOT,
                env={**os.environ,'NX100_HMI_TOKEN':''},stdout=log,stderr=log)
            try:
                base=f'http://127.0.0.1:{port}'
                deadline=time.monotonic()+15
                while True:
                    try:
                        urllib.request.urlopen(base+'/api/state',timeout=1).close()
                        break
                    except OSError:
                        if time.monotonic()>deadline or process.poll() is not None:
                            raise RuntimeError('MOCK browser server unavailable')
                        time.sleep(.1)
                with sync_playwright() as playwright:
                    browser=playwright.chromium.launch(headless=True,channel='msedge')
                    try:
                        page=browser.new_page(viewport=dict(width=1920,height=1080))
                        errors=[]
                        page.on('pageerror',lambda exc:errors.append(str(exc)))
                        page.goto(base)
                        expect(page.locator('#home')).to_be_disabled()
                        expect(page.locator('#connect')).to_be_enabled(timeout=10000)
                        page.locator('#connect').click()
                        expect(page.locator('#home')).to_be_enabled(timeout=10000)
                        page.locator('#home').click()
                        expect(page.locator('#state')).to_have_text('JOB_DISPATCHED')
                        expect(page.locator('#home')).to_be_disabled()
                        expect(page.locator('#start')).to_be_disabled()
                        expect(page.locator('#notice')).to_contain_text('START HOME')
                        expect(page.locator('#ack')).to_be_enabled(timeout=5000)
                        page.screenshot(path=str(report/'home_desktop.png'),full_page=True)
                        page.locator('#ack').click()
                        expect(page.locator('#state')).to_have_text('IDLE')
                        expect(page.locator('#home')).to_be_enabled()
                        page.set_viewport_size(dict(width=1024,height=768))
                        assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
                        page.screenshot(path=str(report/'home_tablet.png'),full_page=True)
                        state=json.loads(journal.read_text(encoding='utf-8'))
                        assert state['job_dispatch']['name']=='HOME'
                        assert state['job_dispatch']['completion_verified'] is False
                        assert not errors,errors
                        result=dict(passed=True,mode='MOCK',home='JOB_DISPATCHED',ack='IDLE',
                                    tablet_no_horizontal_overflow=True,js_errors=errors)
                        (report/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
                        print(json.dumps(result))
                    finally:
                        browser.close()
            finally:
                journal.with_suffix('.stop').write_text('stop',encoding='ascii')
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)


if __name__=='__main__':
    main()
