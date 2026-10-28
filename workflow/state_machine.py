"""Durable transaction intent BEFORE every non-idempotent command."""
import json
import os
import threading
import time
import tempfile
import weakref
from copy import deepcopy

TERMINAL = {'IDLE', 'COMPLETE', 'JOB_DISPATCHED', 'HOLD', 'FAULT'}
STEPS = ['IDLE', 'SERVO_ENABLING', 'SERVO_DISABLING', 'SYSTEM_CHECK', 'WAIT_TARGET', 'TARGET_LOCKED', 'WRITING_PICK_P002', 'PICK_P002_WRITTEN', 'MOVE_TO_SAFE', 'MOVE_XY',
         'XY_VERIFY', 'START_MOVEUNTI', 'Z_DESCENDING', 'LASER_CONFIRMED', 'MOVEUNTI_COMPLETE',
         'VACUUM_ACTIVATING', 'WRITING_DROP_P010', 'DROP_P010_WRITTEN', 'WAIT_DROP_POSITION', 'WAIT_DROP_SAFE', 'DROP_SAFE_REACHED', 'CALCULATE_DROP_Z',
         'MOVING_TO_DROP_Z', 'DROP_Z_REACHED', 'EXECUTING_H2OFF', 'H2OFF_COMPLETE',
         'AWAIT_DROP_CONFIRMATION', 'START_HOME', 'START_PICKDROP', 'PICKDROP_RUNNING', 'PICKDROP_COMPLETE',
         'STACK_COUNT_UPDATED', 'POST_H2OFF_MOVE', 'COMPLETE', 'JOB_DISPATCHED', 'HOLD', 'FAULT']

_PATH_LOCKS = weakref.WeakValueDictionary()
_PATH_LOCKS_GUARD = threading.Lock()
_REPLACE_DELAYS = (.02, .05, .1, .2, .4)


class JournalWriteError(RuntimeError):
    """Persistence failed: no further command intents in this process."""


def _journal_lock(path):
    key = os.path.normcase(str(path.resolve()))
    with _PATH_LOCKS_GUARD:
        lock = _PATH_LOCKS.get(key)
        if lock is None:
            lock = threading.RLock()
            _PATH_LOCKS[key] = lock
        return lock


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
        # Also serialize direct save() and same-path Journal instances. Cycle's
        # existing InstanceLock remains the cross-process controller owner.
        self.lock = _journal_lock(path)
        self.persistence_error = None
        self.state = dict(state='IDLE', revision=0, cycle_id=None, target=None, error='',
                          pending=None, requests=[], history=[], outcome_unknown=False, last_target=None)
        with self.lock:
            if path.exists():
                try:
                    saved = json.loads(path.read_text(encoding='utf-8'))
                    if not isinstance(saved, dict):
                        raise ValueError('Journal must contain an object')
                    self.state.update(saved)
                except (OSError, ValueError, UnicodeError) as exc:
                    # Never overwrite an unreadable/corrupt journal with defaults.
                    self._failed(exc)
                    raise JournalWriteError(self.persistence_error) from exc
                if self.state['state'] not in TERMINAL or self.state['pending']:
                    self.state.update(state='FAULT', error='Server khởi động lại giữa cycle; kiểm tra robot', outcome_unknown=True)
                    self.save()
                # Reading an unchanged terminal journal needs no replace at startup.
            else:
                # An orphan is recovery evidence, never permission to reset IDLE.
                if path.with_suffix('.tmp').exists() or any(path.parent.glob(f'.{path.name}.*.tmp')):
                    self._failed(RuntimeError('Journal missing with temporary recovery files; inspect manually'))
                    raise JournalWriteError(self.persistence_error)
                self.save()

    def _failed(self, exc):
        self.persistence_error = f'Journal persistence failed: {self.path}: {exc}; khóa lệnh mới, kiểm tra file/lock và khởi động lại'
        self.state.update(state='FAULT', error=self.persistence_error, outcome_unknown=True)

    def _healthy(self):
        if self.persistence_error is not None:
            raise JournalWriteError(self.persistence_error)

    def _write(self, candidate):
        self._healthy()
        temp = None
        try:
            # Serialize before touching disk. Unique, same-directory files avoid
            # colliding with old execution.tmp and keep replacement on one volume.
            payload = json.dumps(candidate, ensure_ascii=False, allow_nan=False)
            fd, name = tempfile.mkstemp(prefix=f'.{self.path.name}.{os.getpid()}.', suffix='.tmp', dir=self.path.parent)
            temp = self.path.parent / os.path.basename(name)
            with os.fdopen(fd, 'w', encoding='utf-8') as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            for attempt in range(len(_REPLACE_DELAYS) + 1):
                try:
                    os.replace(temp, self.path)
                    break
                except OSError as exc:
                    transient = isinstance(exc, PermissionError) or getattr(exc, 'winerror', None) in (5, 32, 33)
                    if not transient or attempt == len(_REPLACE_DELAYS):
                        raise
                    # Retry this filesystem operation only, never robot commands.
                    time.sleep(_REPLACE_DELAYS[attempt])
        except Exception as exc:
            self._failed(exc)
            raise JournalWriteError(self.persistence_error) from exc
        finally:
            if temp is not None:
                try:
                    temp.unlink(missing_ok=True)
                except OSError:
                    # Only our unique scratch file; never delete execution.tmp,
                    # the journal, the ownership lock, or operational data.
                    pass

    def save(self):
        with self.lock:
            self._write(self.state)

    def update(self, **values):
        with self.lock:
            self._healthy()
            candidate = deepcopy(self.state)
            candidate.update(values)
            candidate['revision'] += 1
            if 'state' in values:
                candidate['history'] = (candidate['history'] + [dict(state=values['state'], at=time.time())])[-100:]
            self._write(candidate)
            self.state = candidate

    def snapshot(self):
        with self.lock:
            return deepcopy(self.state)

    def intent(self, name, payload):
        self.update(pending=dict(name=name, payload=payload, at=time.time()))

    def accepted(self):
        self.update(pending=None)
