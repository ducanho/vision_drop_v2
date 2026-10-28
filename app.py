"""Local FastAPI HMI. Importing/starting never connects or moves hardware."""
# Run from any selected Python interpreter using this project's dependencies.
# This must happen before importing FastAPI, OpenCV or the REAL detector.
if __name__ == '__main__':
    from pathlib import Path
    import subprocess
    import sys

    entry = Path(__file__).resolve()
    project_python = entry.parent / '.venv' / 'Scripts' / 'python.exe'
    if project_python.is_file() and Path(sys.executable).resolve() != project_python.resolve():
        raise SystemExit(subprocess.call([str(project_python), str(entry), *sys.argv[1:]]))

import argparse
from contextlib import asynccontextmanager
import logging
from logging.handlers import RotatingFileHandler
import os
import secrets
import threading
import time

try:
    from fastapi import FastAPI, Request, HTTPException
    from fastapi.responses import FileResponse, Response, JSONResponse
    from fastapi.staticfiles import StaticFiles
    from starlette.middleware.trustedhost import TrustedHostMiddleware
except ImportError as exc:
    raise SystemExit('Thiếu dependency. Chạy .venv\\Scripts\\python -m pip install -r requirements/base.txt') from exc

from core.settings import ROOT, owned, load_config
from core.launcher import APPLICATION_ID, existing_real_hmi, hmi_url, open_edge, open_when_ready
try:
    from workflow.cycle import Cycle
except ImportError as exc:
    raise SystemExit(f'Thiếu dependency: {exc}. Chạy .venv\\Scripts\\python -m pip install -r requirements/lock.txt') from exc


def create_app(mock=False, config=None, journal_path=None):
    cfg = config or load_config()
    lease = {'id':None, 'expires':0}
    lease_lock = threading.RLock()
    token = os.environ.get('NX100_HMI_TOKEN','')
    if cfg['web']['host'] not in ('127.0.0.1','localhost') and len(token) < 24:
        raise RuntimeError('LAN yêu cầu NX100_HMI_TOKEN tối thiểu 24 ký tự và allowed_clients/allowed_hosts cụ thể')
    cycle = Cycle(cfg,mock,journal_path)

    @asynccontextmanager
    async def lifespan(app):
        yield
        cycle.close()

    app = FastAPI(title='NX100 Auto Pick V1',lifespan=lifespan)
    app.state.cycle = cycle
    app.add_middleware(TrustedHostMiddleware,allowed_hosts=cfg['web']['allowed_hosts'])

    @app.middleware('http')
    async def access(request,call_next):
        if request.client.host not in cfg['web']['allowed_clients']:
            return JSONResponse({'detail':'Client không được phép'},status_code=403)
        if request.method == 'POST':
            if request.headers.get('content-type','').split(';')[0] != 'application/json':
                return JSONResponse({'detail':'Cần application/json'},status_code=415)
            origin = request.headers.get('origin')
            if origin and origin != str(request.base_url).rstrip('/'):
                return JSONResponse({'detail':'Origin bị từ chối'},status_code=403)
        response = await call_next(request)
        response.headers['Cache-Control'] = 'no-store'
        response.headers['X-Content-Type-Options'] = 'nosniff'
        response.headers['Content-Security-Policy'] = "default-src 'self'; img-src 'self' blob:; script-src 'self'; style-src 'self'; frame-ancestors 'none'"
        return response

    @app.exception_handler(RuntimeError)
    async def runtime_error(request,exc):
        return JSONResponse({'detail':str(exc)},status_code=409)

    @app.exception_handler(ValueError)
    async def value_error(request,exc):
        return JSONResponse({'detail':str(exc)},status_code=422)

    def authorized(request, body, stale=True):
        with lease_lock:
            if request.cookies.get('nx100_control') != lease['id'] or not lease['id'] or time.monotonic() >= lease['expires']:
                raise HTTPException(423,'Phiên HMI không giữ quyền điều khiển')
            lease['expires'] = time.monotonic()+30
        with cycle.lock:
            if stale and body.get('revision') != cycle.journal.state['revision']:
                raise HTTPException(409,'Yêu cầu cũ; tải lại trạng thái')
            nonce = body.get('request_id')
            if not isinstance(nonce,str) or not 8 <= len(nonce) <= 100:
                raise HTTPException(422,'request_id không hợp lệ')
            requests = cycle.journal.state.get('api_requests',[])
            if nonce in requests:
                raise HTTPException(409,'Yêu cầu đã xử lý; không tự retry')
            # Persist API admission without changing the operator revision.
            with cycle.journal.lock:
                cycle.journal.state['api_requests'] = requests+[nonce]
                cycle.journal.save()

    @app.get('/')
    def index():
        return FileResponse(owned('templates/index.html'))

    app.mount('/static',StaticFiles(directory=owned('static')),name='static')

    @app.post('/api/control/claim')
    def claim(request:Request, body:dict):
        if token and not secrets.compare_digest(str(body.get('token','')),token):
            raise HTTPException(403,'Mã điều khiển không hợp lệ')
        if request.client.host not in ('127.0.0.1','::1','testclient') and len(token) < 24:
            raise HTTPException(403,'Chưa cấu hình mã điều khiển LAN')
        with lease_lock:
            current = request.cookies.get('nx100_control')
            if lease['id'] and current != lease['id'] and time.monotonic() < lease['expires']:
                raise HTTPException(423,'Một phiên HMI khác đang điều khiển')
            lease.update(id=secrets.token_urlsafe(32),expires=time.monotonic()+30)
            response = JSONResponse({'control':True})
            response.set_cookie('nx100_control',lease['id'],httponly=True,samesite='strict')
            return response

    @app.get('/api/state')
    def state(request:Request):
        with lease_lock:
            control = bool(lease['id'] and request.cookies.get('nx100_control') == lease['id'] and time.monotonic() < lease['expires'])
            if control:
                lease['expires'] = time.monotonic()+30
        return {**cycle.snapshot(),'control':control,'application_id':APPLICATION_ID}

    @app.get('/api/preview.jpg')
    def preview():
        with cycle.vision.lock:
            data = cycle.vision.preview
        if not data or not cycle.vision.status()['connected']:
            raise HTTPException(503,'Camera chưa có frame mới')
        return Response(data,media_type='image/jpeg')

    @app.get('/api/laser/status')
    def laser_status():
        robot = cycle.snapshot()['robot']
        value = robot['laser']
        return {'laser_in1':'UNKNOWN' if value is None else 'ON' if value else 'OFF','mapping':'20050 → IN#(1)',
                'raw_byte':robot.get('laser_raw_byte'),'bit':cycle.robot.laser_config['bit'],
                'mapping_verified':cycle.robot.laser_config['verified'],
                'error':robot.get('read_errors',{}).get('laser','')}

    @app.post('/api/system/connect')
    def connect(request:Request,body:dict):
        with cycle.lock:
            authorized(request,body)
            cycle.connect()
        return cycle.snapshot()

    @app.post('/api/cycle/start')
    def start(request:Request,body:dict):
        with cycle.lock:
            authorized(request,body)
            cycle.start(body['request_id'],body['revision'],body.get('stage','full_cycle'))
        return cycle.snapshot()

    @app.post('/api/robot/home')
    def home(request:Request,body:dict):
        with cycle.lock:
            authorized(request,body)
            cycle.home()
        return cycle.snapshot()

    @app.post('/api/robot/servo-on')
    def servo_on(request:Request,body:dict):
        with cycle.lock:
            authorized(request,body)
        cycle.servo_on()
        return cycle.snapshot()

    @app.post('/api/robot/servo-off')
    def servo_off(request:Request,body:dict):
        with cycle.lock:
            authorized(request,body)
        cycle.servo_off()
        return cycle.snapshot()

    @app.post('/api/cycle/hold')
    def hold(request:Request,body:dict):
        authorized(request,body,stale=False)
        cycle.hold()
        return cycle.snapshot()

    @app.post('/api/cycle/acknowledge')
    def acknowledge(request:Request,body:dict):
        with cycle.lock:
            authorized(request,body)
            cycle.acknowledge(body.get('inspected') is True)
        return cycle.snapshot()

    @app.post('/api/cycle/release-hold')
    def release_hold(request:Request,body:dict):
        with cycle.lock:
            authorized(request,body)
            cycle.release_hold(body.get('confirmed') is True)
        return cycle.snapshot()

    @app.post('/api/vacuum/{channel}')
    def vacuum(channel:int,request:Request,body:dict):
        with cycle.lock:
            authorized(request,body)
            if channel not in (1,2) or type(body.get('on')) is not bool:
                raise ValueError('Cần channel 1/2 và on boolean')
            cycle.vacuum(channel,body['on'])
        return cycle.snapshot()

    @app.post('/api/config')
    def configure(request:Request,body:dict):
        with cycle.lock:
            authorized(request,body)
            cycle.configure(body.get('settings'),body.get('drop_settings'),body.get('speed_i000',...))
        return cycle.snapshot()

    @app.post('/api/drop/settings')
    def drop_settings(request:Request,body:dict):
        with cycle.lock:
            authorized(request,body)
            cycle.configure_drop(body.get('board_thickness_mm'))
        return cycle.snapshot()

    @app.post('/api/drop/count')
    def set_stack_count(request:Request,body:dict):
        with cycle.lock:
            authorized(request,body)
            cycle.set_stack_count(body.get('stacked_count'))
        return cycle.snapshot()

    @app.post('/api/drop/reset')
    def reset_stack(request:Request,body:dict):
        with cycle.lock:
            authorized(request,body)
            cycle.reset_stack(body.get('confirmed') is True)
        return cycle.snapshot()

    @app.post('/api/drop/confirm')
    def confirm_drop(request:Request,body:dict):
        with cycle.lock:
            authorized(request,body)
            cycle.confirm_drop(body.get('cycle_id'),body.get('confirmed') is True)
        return cycle.snapshot()

    return app


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--real',action='store_true',help='REAL is always enabled (compatibility option)')
    parser.add_argument('--no-browser',action='store_true',help='Run REAL without opening Edge')
    parser.add_argument('--port',type=int,help='Override HMI port without changing robot/camera endpoints')
    args = parser.parse_args()
    cfg = load_config()
    if args.port is not None:
        if not 1 <= args.port <= 65535:
            parser.error('--port must be 1..65535')
        cfg['web']['port'] = args.port
    mode = 'real'
    folder = owned(f'logs/{mode}')
    folder.mkdir(parents=True,exist_ok=True)
    handler = RotatingFileHandler(folder/'app.log',maxBytes=2_000_000,backupCount=5,encoding='utf-8')
    logging.basicConfig(level=logging.INFO,handlers=[handler,logging.StreamHandler()])
    url=hmi_url(cfg['web']['host'],cfg['web']['port'])
    if existing_real_hmi(url):
        logging.info('HMI REAL đã chạy tại %s',url)
        if not args.no_browser:
            open_edge(url)
        return
    import uvicorn
    server=uvicorn.Server(uvicorn.Config(create_app(mock=False,config=cfg),host=cfg['web']['host'],port=cfg['web']['port']))
    if not args.no_browser:
        threading.Thread(target=open_when_ready,args=(server,url),daemon=True,name='open-edge').start()
    server.run()


if __name__ == '__main__':
    main()
