"""Open the operator HMI in Edge after Uvicorn has bound its socket."""
import json
import logging
import os
from pathlib import Path
import shutil
import subprocess
import time
import urllib.request

APPLICATION_ID='nx100_full_cycle_v1'


def hmi_url(host,port):
    return f'http://{"127.0.0.1" if host in ("0.0.0.0","::") else host}:{port}'


def existing_real_hmi(url):
    try:
        with urllib.request.urlopen(url+'/api/state',timeout=5) as response:
            state=json.load(response)
        return state.get('application_id')==APPLICATION_ID and state.get('mode')=='REAL'
    except (OSError,ValueError):
        return False


def open_edge(url):
    executable=shutil.which('msedge')
    if not executable:
        for variable in ('PROGRAMFILES(X86)','PROGRAMFILES','LOCALAPPDATA'):
            base=os.environ.get(variable)
            if base:
                candidate=Path(base)/'Microsoft/Edge/Application/msedge.exe'
                if candidate.is_file():
                    executable=str(candidate)
                    break
    if not executable:
        logging.warning('Không tìm thấy Microsoft Edge. Mở giao diện tại %s',url)
        return
    try:
        subprocess.Popen([executable,'--new-tab',url])
    except OSError:
        logging.exception('Không mở được Edge. Giao diện: %s',url)


def open_when_ready(server,url):
    while not server.started:
        if server.should_exit:
            return
        time.sleep(.1)
    open_edge(url)
