"""Launch a private MOCK server, exercise HMI, export desktop/tablet screenshots."""
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import urllib.request
from playwright.sync_api import sync_playwright,expect
from core.settings import ROOT,owned


def wait_vacuum(page,base,on):
    deadline=time.monotonic()+5
    while time.monotonic()<deadline:
        if page.request.get(base+'/api/state').json()['robot']['vacuum'][0] is on:
            return
        page.wait_for_timeout(50)
    raise AssertionError('Manual vacuum did not reach expected state')


def main():
    report_dir=owned('logs/reports/browser')
    report_dir.mkdir(parents=True,exist_ok=True)
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',0))
        port=sock.getsockname()[1]
    with tempfile.TemporaryDirectory(prefix='hmi-') as temporary:
        command="""
from app import create_app
import uvicorn,sys,time,threading
from pathlib import Path
journal=Path(sys.argv[1])
app=create_app(mock=True,journal_path=journal)
app.state.cycle.cfg['workflow']['repeat_until_empty']=False
app.state.cycle.robot.client.pressure_inputs=[True,False]
server=uvicorn.Server(uvicorn.Config(app,host='127.0.0.1',port=int(sys.argv[2]),log_level='error'))
def watch():
 while not journal.with_suffix('.stop').exists(): time.sleep(.1)
 server.should_exit=True
threading.Thread(target=watch,daemon=True).start()
server.run()
"""
        log=(report_dir/'browser_server.log').open('w',encoding='utf-8')
        process=subprocess.Popen([sys.executable,'-c',command,str(Path(temporary)/'execution.json'),str(port)],cwd=ROOT,
                                 env={**os.environ,'NX100_HMI_TOKEN':''},stdout=log,stderr=log)
        base=f'http://127.0.0.1:{port}'
        try:
            deadline=time.monotonic()+15
            while True:
                try:
                    urllib.request.urlopen(base+'/api/state',timeout=1).close()
                    break
                except Exception:
                    if time.monotonic()>deadline:
                        raise RuntimeError('Browser server did not start')
                    time.sleep(.1)
            with sync_playwright() as p:
                browser=p.chromium.launch(headless=True,channel='msedge')
                page=browser.new_page(viewport={'width':1920,'height':1080},device_scale_factor=1)
                errors=[]
                page.on('pageerror',lambda error:errors.append(str(error)))
                page.goto(base)
                expect(page.locator('#vacuumInput1, #vacuumInput2, #v1, #v2')).to_have_count(0)
                expect(page.locator('#claim, #controlToken, #stage, #progress')).to_have_count(0)
                expect(page.locator('#connect')).to_be_enabled(timeout=10000)
                page.get_by_role('button',name='CONNECT').click()
                expect(page.locator('#ready')).to_have_text('READY',timeout=10000)
                expect(page.locator('#pressure1')).to_have_text('ON')
                expect(page.locator('#pressure2')).to_have_text('OFF')
                assert page.evaluate('document.documentElement.scrollHeight<=window.innerHeight')
                page.screenshot(path=str(report_dir/'hmi_desktop_ready.png'),full_page=True)
                page.get_by_role('button',name='▶ START',exact=True).click()
                expect(page.locator('#state')).to_have_text('COMPLETE',timeout=15000)
                expect(page.locator('#confirmDrop')).to_have_count(0)
                expect(page.locator('#stackedCount')).to_have_value('1')
                assert page.request.get(base+'/api/state').json()['drop_height']['next_drop_z_mm']==342
                expect(page.locator('#pos-z')).to_have_text('530.00')
                snapshot=page.request.get(base+'/api/state').json()
                assert snapshot['moveunti_completion']=={'criteria':'laser_on_and_stopped','laser':True,'running':False}
                assert snapshot['vacuum_job']=={'name':'H2ON','status':'complete'}
                assert snapshot['settings']['movl_speed']==80
                assert page.locator('#start').is_disabled()
                page.screenshot(path=str(report_dir/'hmi_desktop_complete.png'),full_page=True)
                page.locator('#stackedCount').fill('7')
                page.wait_for_timeout(1100)
                expect(page.locator('#stackedCount')).to_have_value('7')
                assert page.request.get(base+'/api/state').json()['drop_height']['stacked_count']==1
                page.locator('#saveStackCount').click()
                expect(page.locator('#stackedCount')).to_have_value('7')
                expect(page.locator('#saveStackCount')).to_be_enabled()
                assert page.request.get(base+'/api/state').json()['drop_height']['next_drop_z_mm']==414
                page.locator('#stackedCount').fill('0')
                page.locator('#saveStackCount').click()
                expect(page.locator('#saveStackCount')).to_be_enabled()
                expect(page.locator('#stackedCount')).to_have_value('0')
                assert page.request.get(base+'/api/state').json()['drop_height']['next_drop_z_mm']==330
                expect(page.locator('#dropThickness')).to_have_value('12')
                page.get_by_role('button',name='RESET').click()
                expect(page.locator('#state')).to_have_text('IDLE')
                page.locator('[data-channel="1"][data-on="false"]').click()
                wait_vacuum(page,base,False)
                page.locator('[data-channel="1"][data-on="true"]').click()
                wait_vacuum(page,base,True)
                page.get_by_role('button',name='HOLD').click()
                expect(page.locator('#state')).to_have_text('HOLD')
                expect(page.locator('#hold')).to_have_text('UNHOLD')
                expect(page.locator('#hold')).to_be_disabled()
                expect(page.locator('#releaseHold, #inspected')).to_have_count(0)
                page.get_by_role('button',name='RESET').click()
                expect(page.locator('#state')).to_have_text('IDLE')
                expect(page.locator('#hold')).to_be_enabled()
                page.once('dialog',lambda dialog:dialog.dismiss())
                page.locator('#hold').click()
                expect(page.locator('#flag-hold')).to_have_text('ON')
                page.once('dialog',lambda dialog:dialog.accept())
                page.locator('#hold').click()
                expect(page.locator('#flag-hold')).to_have_text('OFF')
                expect(page.locator('#hold')).to_have_text('HOLD')
                page.set_viewport_size({'width':1024,'height':768})
                page.screenshot(path=str(report_dir/'hmi_tablet.png'),full_page=True)
                assert page.evaluate('document.documentElement.scrollWidth<=window.innerWidth')
                assert not errors,errors
                result={'desktop':'1920x1080','desktop_fits_viewport':True,'tablet':'1024x768','tablet_no_horizontal_overflow':True,
                        'cycle':'COMPLETE','automatic_h2on':True,'z_completion':'laser_on_and_stopped','default_speed':80,'manual_vacuum1':'ON','release_hold':'OFF','h2off':'complete','automatic_count':True,'post_drop_z':530,'stacked_count':1,'next_drop_z':342,'reset_stack_count':0,'reset_next_drop_z':330,'js_errors':errors}
                (report_dir/'browser_result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
                print(json.dumps(result))
                browser.close()
        finally:
            (Path(temporary)/'execution.stop').write_text('stop',encoding='ascii')
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                if os.name=='nt':
                    subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],capture_output=True)
                else:
                    process.kill()
                process.wait(timeout=5)
            log.close()


if __name__=='__main__':
    main()
