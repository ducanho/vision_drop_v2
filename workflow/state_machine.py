"""Durable transaction intent BEFORE every non-idempotent command."""
import json
import os
import threading
import time
from copy import deepcopy

TERMINAL = {'IDLE', 'COMPLETE', 'JOB_DISPATCHED', 'HOLD', 'FAULT'}
STEPS = ['IDLE', 'SERVO_ENABLING', 'SERVO_DISABLING', 'SYSTEM_CHECK', 'WAIT_TARGET', 'TARGET_LOCKED', 'WRITING_PICK_P002', 'PICK_P002_WRITTEN', 'MOVE_TO_SAFE', 'MOVE_XY',
         'XY_VERIFY', 'START_MOVEUNTI', 'Z_DESCENDING', 'LASER_CONFIRMED', 'MOVEUNTI_COMPLETE',
         'VACUUM_ACTIVATING', 'WRITING_DROP_P010', 'DROP_P010_WRITTEN', 'WAIT_DROP_POSITION', 'WAIT_DROP_SAFE', 'DROP_SAFE_REACHED', 'CALCULATE_DROP_Z',
         'MOVING_TO_DROP_Z', 'DROP_Z_REACHED', 'EXECUTING_H2OFF', 'H2OFF_COMPLETE',
         'AWAIT_DROP_CONFIRMATION', 'STACK_COUNT_UPDATED', 'POST_H2OFF_MOVE', 'COMPLETE', 'JOB_DISPATCHED', 'HOLD', 'FAULT']


class InstanceLock:
    """OS-owned lock, released automatically after process termination."""
    def __init__(self,path):
        path.parent.mkdir(parents=True,exist_ok=True)
        self.stream=path.open('a+b')
        self.stream.seek(0,2)
        if self.stream.tell()==0:
            self.stream.write(b'0')
            self.stream.flush()
        self.stream.seek(0)
        try:
            if os.name=='nt':
                import msvcrt
                msvcrt.locking(self.stream.fileno(),msvcrt.LK_NBLCK,1)
            else:
                import fcntl
                fcntl.flock(self.stream.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        except OSError as exc:
            self.stream.close()
            self.stream=None
            raise RuntimeError('Mode này đã có server đang giữ quyền điều khiển; không chạy process thứ hai') from exc

    def close(self):
        if self.stream:
            self.stream.close()
            self.stream=None

    def __del__(self):
        self.close()


class Journal:
    def __init__(self, path):
        self.path = path
        path.parent.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.state = dict(state='IDLE', revision=0, cycle_id=None, target=None, error='',
                          pending=None, requests=[], history=[], outcome_unknown=False, last_target=None)
        if path.exists():
            try:
                self.state.update(json.loads(path.read_text(encoding='utf-8')))
            except Exception:
                self.state.update(state='FAULT', error='Journal hỏng; cần kiểm tra thủ công', outcome_unknown=True)
            if self.state['state'] not in TERMINAL or self.state['pending']:
                self.state.update(state='FAULT', error='Server khởi động lại giữa cycle; kiểm tra robot', outcome_unknown=True)
        self.save()

    def save(self):
        temp = self.path.with_suffix('.tmp')
        with temp.open('w', encoding='utf-8') as stream:
            json.dump(self.state, stream, ensure_ascii=False, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp, self.path)

    def update(self, **values):
        with self.lock:
            self.state.update(values)
            self.state['revision'] += 1
            if 'state' in values:
                self.state['history'] = (self.state['history'] + [dict(state=values['state'], at=time.time())])[-100:]
            self.save()

    def snapshot(self):
        with self.lock:
            return deepcopy(self.state)

    def intent(self, name, payload):
        self.update(pending=dict(name=name, payload=payload, at=time.time()))

    def accepted(self):
        self.update(pending=None)
