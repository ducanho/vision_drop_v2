import logging
import math
import threading
import time
import uuid
from copy import deepcopy
from core.settings import AXES, owned, validate_settings
from robot.controller import Controller
from robot import job_control
from vision.pipeline import Vision
from vision.stability import StableTarget
from .state_machine import Journal, InstanceLock
from .safety_checks import ready, workspace, reached, real_gate, manual_io_ready, hold_release_ready, servo_on_ready, servo_off_ready, operator_motion_workflow
from robot.vacuum import validate_mapping
from robot.position_variable import pick_position,verify_p002
from .drop import DropWorkflow,StackStore


class Cycle(DropWorkflow):
    def __init__(self, cfg, mock=True, journal_path=None):
        self.cfg, self.mock = deepcopy(cfg), mock
        self.lock = threading.RLock()
        self.cancel = threading.Event()
        self.robot = Controller(self.cfg, mock)
        self.vision = Vision(self.cfg, mock)
        journal_path = journal_path or owned(f'data/{"mock" if mock else "real"}/execution.json')
        self.instance_lock = InstanceLock(journal_path.with_suffix('.lock'))
        self.journal = Journal(journal_path)
        self.stack=StackStore(journal_path.parent/'drop_stack.json',self.cfg.get('drop',{}))
        saved_settings = self.journal.path.parent/'operator_settings.json'
        if saved_settings.exists():
            import json
            self.cfg['settings'].update(validate_settings(json.loads(saved_settings.read_text(encoding='utf-8'))))
        self.thread = None
        self.telemetry = dict(connected=False, flags={k:None for k in ('servo','remote','play','teach','alarm','error','running','hold')},
                              position=None, laser=None, vacuum=[None,None], error='Chưa kết nối')
        self.telemetry_stamp = 0
        self.stopping = threading.Event()
        self.poller = None
        self.monitor_requested = False
        self.prefetch = dict(state='IDLE', cycle_id=None, b090=None)
        self._validated_prefetch_identity = None

    @property
    def busy(self):
        return self.thread is not None and self.thread.is_alive()

    def connect(self):
        with self.lock:
            if self.busy:
                raise RuntimeError('Cycle đang chạy')
            self.robot.connect()
            self.monitor_requested = True
            if not self.poller:
                self.poller = threading.Thread(target=self._poll, daemon=True, name='telemetry')
                self.poller.start()
            self.refresh()
            self.vision.connect()

    def _poll(self):
        while not self.stopping.wait(1.0):
            if self.monitor_requested:
                try:
                    self.refresh()
                except Exception:
                    pass

    def refresh(self):
        try:
            data = self.robot.observe()
            with self.lock:
                self.telemetry = data
                self.telemetry_stamp = time.monotonic()
            return data
        except Exception as exc:
            self.robot.connected = False
            with self.lock:
                self.telemetry.update(connected=False, flags={k:None for k in self.telemetry['flags']},
                                      position=None, laser=None, laser_raw_byte=None, vacuum=[None,None],vacuum_raw_byte=None,
                                      vacuum_inputs=[None,None],
                                      read_errors={'laser':str(exc)},error=str(exc))
            raise

    def snapshot(self):
        with self.lock:
            result = self.journal.snapshot()
            t = deepcopy(self.telemetry)
            t['vacuum'] = self.robot.vacuum[:] if t['connected'] else [None,None]
            age = time.monotonic()-self.telemetry_stamp
            if age > 2:
                t.update(connected=False, flags={k:None for k in t['flags']}, position=None, laser=None,
                         laser_raw_byte=None, vacuum=[None,None],vacuum_raw_byte=None,vacuum_inputs=[None,None])
            result.update(robot=t, camera=self.vision.status(), mode='MOCK' if self.mock else 'REAL', busy=self.busy,
                          settings=deepcopy(self.cfg['settings']), telemetry_age=round(age,2),
                          vacuum_mapping=self.robot.vacuum_config['mode'],
                          vacuum_method=self.robot.vacuum_config.get('method','network'),
                          vacuum_jobs=deepcopy(self.robot.vacuum_config.get('jobs',{})),
                          end_after_moveunti_start=self.cfg.get('workflow',{}).get('end_after_moveunti_start',False),
                          z_completion=self.cfg.get('workflow',{}).get('z_completion'),
                          real_limit='Chưa có tín hiệu chứng minh nguyên nhân dừng JOB; REAL không tự kích vacuum sau MOVEUNTI' if not self.mock and not operator_motion_workflow(self.cfg) else '')
            result['capabilities'] = {}
            for stage in ('full_cycle','xy_only','moveunti_only','vacuum_only'):
                reason = ''
                if not self.mock:
                    try:
                        real_gate(self.cfg,stage)
                    except RuntimeError as exc:
                        reason = str(exc)
                result['capabilities'][stage] = dict(allowed=not reason,reason=reason)
            reason=''
            try:
                validate_mapping(self.robot.vacuum_config)
                if not t['connected']:
                    raise RuntimeError('Kết nối robot trước khi điều khiển vacuum')
                manual_io_ready(t['flags'])
            except RuntimeError as exc:
                reason=str(exc)
            result['capabilities']['manual_vacuum']=dict(allowed=not reason,reason=reason)
            reason=''
            try:
                self._hold_release_state_ready()
                if not t['connected']:
                    raise RuntimeError('Kết nối robot trước khi nhả HOLD')
                hold_release_ready(t['flags'])
            except RuntimeError as exc:
                reason=str(exc)
            result['capabilities']['release_hold']=dict(allowed=not reason,reason=reason)
            for action,check in (('servo_on',servo_on_ready),('servo_off',servo_off_ready)):
                reason=''
                try:
                    self._servo_state_ready()
                    if not t['connected']:
                        raise RuntimeError('Kết nối robot trước khi điều khiển servo')
                    check(t['flags'])
                except RuntimeError as exc:
                    reason=str(exc)
                result['capabilities'][action]=dict(allowed=not reason,reason=reason)
            result['drop_height']={**self.stack.snapshot(),'can_confirm':self._can_confirm_drop(),
                                   'cycle_id':self.journal.state['cycle_id']}
            result['prefetch']={**deepcopy(self.prefetch),'vision':self.vision.prefetch_status()}
            return result

    def _step(self, state, **values):
        with self.lock:
            self._check_cancel()
            self.journal.update(state=state, **values)
            logging.getLogger('cycle').info('%s %s', self.journal.state['cycle_id'], state)

    def _check_cancel(self):
        if self.cancel.is_set() or self.stopping.is_set():
            raise RuntimeError('Cycle đã bị HOLD/dừng; không tự resume')

    def _command(self, name, payload, action):
        with self.lock:
            self._check_cancel()
            self.journal.intent(name, payload)
            action()
            self.journal.accepted()

    def start(self, request_id, revision, stage='full_cycle'):
        with self.lock:
            if not isinstance(request_id, str) or not 8 <= len(request_id) <= 100:
                raise ValueError('request_id không hợp lệ')
            state = self.journal.snapshot()
            if request_id in state['requests']:
                raise RuntimeError('Duplicate START; không gửi lại')
            if revision != state['revision']:
                raise RuntimeError('Yêu cầu cũ; tải lại trạng thái HMI')
            if self.busy or state['state'] != 'IDLE':
                raise RuntimeError('Cần ACK; chỉ một cycle được phép chạy')
            if stage not in ('full_cycle','xy_only','moveunti_only','vacuum_only'):
                raise ValueError('Stage không hợp lệ')
            if stage=='vacuum_only' or (stage=='full_cycle' and not self.cfg.get('workflow',{}).get('end_after_moveunti_start',False)):
                validate_mapping(self.cfg['vacuum'])
                if self.cfg['vacuum'].get('method')!='job':
                    raise RuntimeError('V1 yêu cầu JOB HAND 2 ON đã xác nhận')
            if not self.mock:
                real_gate(self.cfg, stage)
            if not self.robot.connected or not self.vision.status()['connected']:
                raise RuntimeError('Kết nối robot và camera, chờ frame mới')
            ready(self.robot.status())
            if self.robot.laser() is not False and stage != 'vacuum_only':
                raise RuntimeError('Laser phải OFF đã xác minh trước cycle')
            self.cancel.clear()
            self.vision.clear_next_pick()
            self.journal.update(state='SYSTEM_CHECK', cycle_id=uuid.uuid4().hex, target=None, error='',
                                requests=state['requests']+[request_id], pending=None, outcome_unknown=False,
                                vacuum_job=None,manual_job=None,job_dispatch=None,job_start_attempted=False,moveunti_completion=None,
                                drop_release=None,drop_target=None,drop_safe_pose=None,drop_plan=None,drop_attempt_cycle_id=None,
                                stop_reason=None,post_drop_target=None,drop_variable=None,pick_variable=None)
            self.thread = threading.Thread(target=self._run, args=(stage,), daemon=True, name='pick-cycle')
            self.thread.start()

    def _wait(self, seconds):
        if self.cancel.wait(seconds):
            self._check_cancel()
        self._check_cancel()

    def _prefetch_event(self, event):
        logging.getLogger('cycle').info('%s %s',self.prefetch['cycle_id'],event)

    def _arm_drop_prefetch(self):
        self.vision.clear_next_pick()
        self.prefetch=dict(state='WAITING_B090',cycle_id=self.journal.state['cycle_id'],
                           b090=None,reset_confirmed=False,reason=None,
                           connection_generation=self.robot.connection_generation)
        self._prefetch_event('DROP_STARTED')
        self._poll_drop_prefetch()

    def _poll_drop_prefetch(self):
        """Only the cycle worker polls; a START ACK alone cannot prove a reset."""
        if self.prefetch['state']!='WAITING_B090':
            return
        try:
            with self.lock,self.robot.lock:
                self._check_cancel()
                if self.prefetch['cycle_id']!=self.journal.state['cycle_id']:
                    raise RuntimeError('DROP cycle changed')
                if self.prefetch['connection_generation']!=self.robot.connection_generation:
                    raise RuntimeError('NX100 reconnected')
                if not self.robot.connected or self.robot.current_job()!='DROP':
                    raise RuntimeError('DROP connection/job unverified')
                value=self.robot.read_b090()
                self.prefetch['b090']=value
                if value==0 and not self.prefetch['reset_confirmed']:
                    self.prefetch['reset_confirmed']=True
                    self._prefetch_event('B090_RESET_CONFIRMED')
                elif value==1 and self.prefetch['reset_confirmed']:
                    self._prefetch_event('B090_TRIGGERED')
                    self.prefetch['state']='DETECTING'
                    self._prefetch_next_pick()
                elif value not in (0,1):
                    raise RuntimeError('B090 is not a checkpoint value')
        except Exception as exc:
            self.prefetch.update(state='INVALID',b090=None,reason=str(exc))
            self.vision.clear_next_pick(str(exc))
            self._prefetch_event('PREFETCH_INVALID')

    def _prefetch_next_pick(self):
        target=self.journal.state.get('target') or {}
        if (self.prefetch['state']=='DETECTING' and self.journal.state.get('automatic_repeat')
                and target.get('id') is not None):
            self.vision.prepare_next_pick(after_id=target['id'],cycle_id=self.prefetch['cycle_id'])
            self._prefetch_event('PREFETCH_STARTED')

    def _target(self):
        self._step('WAIT_TARGET')
        fresh_after = time.monotonic()
        self._validated_prefetch_identity = None
        cursor=self.stack.snapshot().get('last_picked_id')
        context=self.vision.prefetch_status()
        if context.get('cycle_id') is not None and (not self.robot.connected or
                self.prefetch.get('connection_generation')!=self.robot.connection_generation):
            self.vision.clear_next_pick('NX100 connection changed')
        if context.get('cycle_id') is not None and context['cycle_id']!=self.stack.snapshot()['last_completed_drop_cycle_id']:
            self.vision.clear_next_pick('wrong source cycle')
        saved=self.vision.take_ready_pick(after_id=cursor,cycle_id=self.stack.snapshot()['last_completed_drop_cycle_id']) if self.journal.state.get('automatic_repeat') else None
        if saved is not None:
            self._validated_prefetch_identity=self.vision.preparation_identity()
            workspace(saved['pose'],self.cfg)
            self._step('TARGET_LOCKED',target={**saved,'cycle_id':self.journal.state['cycle_id'],
                       'prefetch_cycle_id':context.get('cycle_id')},
                       target_prefilled_frames=saved['prepared_frames'],
                       target_wait_seconds=round(time.monotonic()-fresh_after,3))
            self._prefetch_event('NEXT_TARGET_LOADED')
            return saved['pose']
        prepared=self.vision.take_next_pick(after_id=cursor) if self.journal.state.get('automatic_repeat') else None
        prepared_identity=self.vision.preparation_identity() if prepared is not None else None
        stable=prepared if prepared is not None else StableTarget(self.cfg['vision'],after_id=cursor)
        prefilled=len(stable.samples)
        empty_since = None
        empty_samples = 0
        previous_frame = -1
        previous_stamp = 0
        deadline = time.monotonic()+self.cfg['motion']['target_timeout']
        while time.monotonic() < deadline:
            self._check_cancel()
            ready(self.robot.status())
            if prepared is not None and prepared_identity!=self.vision.preparation_identity():
                self.vision.clear_next_pick('calibration/config changed during validation')
                prepared=None
                stable=StableTarget(deepcopy(self.cfg['vision']),after_id=cursor)
                prefilled=0
                fresh_after=time.monotonic()
            frame,stamp,items = self.vision.sample()
            # Even with prepared history, require a NEW post-lift frame to validate it.
            if self.journal.state.get('automatic_repeat'):
                if stamp <= fresh_after or frame <= previous_frame or stamp <= previous_stamp:
                    self._wait(self.cfg['motion']['poll_seconds'])
                    continue
                if stamp-previous_stamp > self.cfg['vision']['max_frame_gap']:
                    empty_since = None
                    empty_samples = 0
                previous_frame, previous_stamp = frame, stamp
                if not items:
                    empty_since = stamp if empty_since is None else empty_since
                    empty_samples += 1
                    if (empty_samples >= self.cfg['vision']['stable_frames'] and
                        stamp-empty_since >= self.cfg['workflow'].get('empty_seconds',2)):
                        return None
                else:
                    empty_since = None
                    empty_samples = 0
            target = stable.update(frame, stamp, items, time.monotonic())
            if target:
                settings = self.cfg['settings']
                pose = dict(zip(AXES, (*target['xyz'][:2], settings['z_safe'], settings['rx'],settings['ry'],settings['rz'])))
                workspace(pose,self.cfg)
                if prepared is not None:
                    self._validated_prefetch_identity = prepared_identity
                    self._prefetch_event('NEXT_TARGET_VALIDATED')
                self._step('TARGET_LOCKED', target={**target,'pose':pose,'frame_id':frame,
                           'cycle_id':self.journal.state['cycle_id'],
                           'prefetch_cycle_id':context.get('cycle_id') if prepared is not None else None},
                           target_prefilled_frames=prefilled,target_wait_seconds=round(time.monotonic()-fresh_after,3))
                return pose
            self._wait(self.cfg['motion']['poll_seconds'])
        raise RuntimeError('Hết thời gian chờ target ổn định')

    def _pose_checks(self, reference):
        if operator_motion_workflow(self.cfg):
            return  # Preserve live RPOSC metadata; no static expected-tool gate.
        if not self.mock:
            for index,key in [(6,'expected_posture'),(7,'expected_tool')]:
                if int(reference.fields[index]) != self.cfg['motion'][key]:
                    raise RuntimeError(f'{key} thay đổi')

    def _move(self, target, state, required_start=None):
        if state=='MOVE_XY':
            raise RuntimeError('MOVE_XY phải ghi/verify P002; không gửi MOVL XY từ PC')
        self._step(state)
        with self.lock, self.robot.lock:
            self._check_cancel()
            ready(self.robot.status())
            start, reference = self.robot.pose()
            if required_start is not None and not reached(start,required_start,self.cfg):
                raise RuntimeError('DROP SAFE đã thay đổi trước khi hạ Z; không gửi MOVL')
            self._pose_checks(reference)
            workspace(start,self.cfg)
            workspace(target,self.cfg)
            self._command('MOVL',target,lambda:self.robot.move(target,self.cfg['settings']['movl_speed'],reference))
        deadline = time.monotonic()+self.cfg['motion']['timeout']
        settled = 0
        while time.monotonic() < deadline:
            self._check_cancel()
            data = self.refresh()
            ready(data['flags'],allow_running=True)
            _, current_reference = self.robot.pose()
            if current_reference.fields[6:] != reference.fields[6:]:
                raise RuntimeError('Tool/posture/trục ngoài thay đổi trong MOVL')
            settled = settled+1 if not data['flags']['running'] and reached(data['position'],target,self.cfg) else 0
            if settled >= self.cfg['motion']['settle_samples']:
                return
            self._wait(self.cfg['motion']['poll_seconds'])
        raise RuntimeError('MOVL timeout; ACK không chứng minh đã tới đích')

    def _check_pick_prefetch(self):
        if self._validated_prefetch_identity is not None:
            if not self.vision.connected:
                raise RuntimeError('Camera disconnected before P002/START')
            if (not self.robot.connected or
                    self.prefetch.get('connection_generation')!=self.robot.connection_generation or
                    self._validated_prefetch_identity!=self.vision.preparation_identity()):
                raise RuntimeError('Prefetch identity/connection changed before P002/START')

    def _prepare_pick_variable(self,target):
        with self.lock,self.robot.lock:
            self._check_cancel()
            if not self.robot.connected:
                raise RuntimeError('Robot disconnected; không ghi P002')
            ready(self.robot.status())
            self._check_pick_prefetch()
            start,reference=self.robot.pose()
            self._pose_checks(reference)
            workspace(start,self.cfg)
            settings=self.cfg['settings']
            pose={**target,'z':settings['z_safe'],**{a:settings[a] for a in ('rx','ry','rz')}}
            workspace(pose,self.cfg)
            fields,payload=pick_position(pose,reference)
            receipt=dict(expected=list(fields),metadata=list(reference.fields[6:]),verified=False,
                         cycle_id=self.journal.state['cycle_id'],connection_generation=self.robot.connection_generation)
            self._step('WRITING_PICK_P002',pick_variable=receipt)
            self._command('LOADV',payload,lambda:self.robot.client.execute('LOADV',payload))
            verify_p002(fields,self.robot.client.execute('SAVEV','4,2'))
            self._step('PICK_P002_WRITTEN',pick_variable={**receipt,'verified':True})

    def _start_moveunti(self):
        self._step('START_MOVEUNTI')
        with self.lock, self.robot.lock:
            self._check_cancel()
            ready(self.robot.status())
            self._check_pick_prefetch()
            receipt=self.journal.state.get('pick_variable') or {}
            if (not self.robot.connected or not receipt.get('verified') or receipt.get('cycle_id')!=self.journal.state['cycle_id'] or
                    receipt.get('connection_generation')!=self.robot.connection_generation):
                raise RuntimeError('Chưa xác minh P002 cho cycle/kết nối hiện tại; không START MOVEUNTI')
            verify_p002(receipt['expected'],self.robot.client.execute('SAVEV','4,2'))
            if not job_control.exists(self.robot.client):
                raise RuntimeError('JOB MOVEUNTI không tồn tại')
            if self.robot.laser() is not False:
                raise RuntimeError('Laser ON/UNKNOWN trước Z; khóa START JOB')
            current,reference = self.robot.pose()
            workspace(current,self.cfg)
            if list(reference.fields[6:])!=receipt['metadata']:
                raise RuntimeError('Tool/posture/trục ngoài thay đổi sau ghi P002; không START MOVEUNTI')
            start=dict(zip(AXES,map(float,receipt['expected'][2:8])))
            workspace(start,self.cfg)
            self._check_pick_prefetch()
            self.journal.update(job_start_attempted=True)
            self._command('START','MOVEUNTI',lambda:job_control.start(self.robot.client))
            if self.vision.prefetch_status().get('state') in ('PREPARED','VALIDATING'):
                self.prefetch['state']='CONSUMED'
                self.vision.consume_next_pick()
                self._prefetch_event('NEXT_MOVEUNTI_SENT')
        return start

    def _job(self):
        start=self._start_moveunti()
        if self.cfg.get('workflow',{}).get('z_completion')=='laser_on_and_stopped':
            return self._wait_laser_and_stopped(start)
        self._step('Z_DESCENDING')
        observed = dict(running_seen=False,laser_edge=False)
        deadline = time.monotonic()+self.cfg['motion']['job_timeout']
        while time.monotonic() < deadline:
            self._check_cancel()
            data = self.refresh()
            ready(data['flags'],allow_running=True)
            workspace(data['position'],self.cfg)
            job_control.current(self.robot.client)
            if data['flags']['running']:
                observed['running_seen'] = True
            if data['laser'] is None:
                raise RuntimeError('Laser UNKNOWN khi JOB đang chạy')
            if data['laser'] and not observed['laser_edge']:
                if not observed['running_seen']:
                    raise RuntimeError('Không quan sát RUNNING trước laser; cần kiểm tra')
                observed['laser_edge'] = True
                self._step('LASER_CONFIRMED')
            p = data['position']
            if data['flags']['running'] is False:
                if any(abs((p[a]-start[a]+180)%360-180)>self.cfg['motion']['angle_tolerance'] for a in ('rx','ry','rz')):
                    raise RuntimeError('Orientation thay đổi trong MOVEUNTI')
                if math.dist((p['x'],p['y']),(start['x'],start['y'])) > self.cfg['motion']['xyz_tolerance']:
                    raise RuntimeError('XY thay đổi trong MOVEUNTI')
                if not self.mock:
                    j = self.cfg['job']
                    if p['z'] < j['min_z'] or start['z']-p['z'] > j['max_descent_mm'] or p['z'] > start['z']+1:
                        raise RuntimeError('MOVEUNTI ngoài hành trình Z đã xác minh')
            if observed['running_seen'] and not data['flags']['running']:
                if not job_control.stop_reason_proven(self.robot,observed):
                    raise RuntimeError('Chưa chứng minh JOB dừng do laser; khóa chuyển bước/vacuum, kiểm tra thủ công')
                if not data['laser'] or p['z'] >= start['z']-self.cfg['motion']['xyz_tolerance']:
                    raise RuntimeError('Không có laser/descent hợp lệ')
                self._step('MOVEUNTI_COMPLETE',stop_position=p)
                return
            self._wait(self.cfg['motion']['poll_seconds'])
        raise RuntimeError('MOVEUNTI/laser timeout; robot có thể vẫn đang chạy')

    def _wait_laser_and_stopped(self,start):
        self._step('Z_DESCENDING')
        deadline=time.monotonic()+self.cfg['motion']['job_timeout']
        settled=0
        laser_seen=False
        while time.monotonic()<deadline:
            self._check_cancel()
            data=self.refresh()
            ready(data['flags'],allow_running=True)
            workspace(data['position'],self.cfg)
            if data['laser'] is None:
                raise RuntimeError('Laser UNKNOWN khi chờ MOVEUNTI; không DROP')
            current_job=self.robot.current_job()
            if current_job==self.cfg['vacuum']['jobs']['2_on']:
                # A CALL to H2ON inside MOVEUNTI is not parent JOB completion.
                settled=0
                self._wait(self.cfg['motion']['poll_seconds'])
                continue
            if current_job!='MOVEUNTI':
                raise RuntimeError('JOB hiện tại không còn là MOVEUNTI; không DROP')
            p=data['position']
            # Operator-confirmed JOB order: MOVL P002, then Z search. Do not
            # require polling to catch the intermediate P002 pose.
            completed=data['laser'] is True and data['flags']['running'] is False
            if completed:
                if math.dist((p['x'],p['y']),(start['x'],start['y']))>self.cfg['motion']['xyz_tolerance']:
                    raise RuntimeError('XY khi MOVEUNTI dừng không khớp P002; không DROP')
                if any(abs((p[a]-start[a]+180)%360-180)>self.cfg['motion']['angle_tolerance'] for a in ('rx','ry','rz')):
                    raise RuntimeError('Orientation khi MOVEUNTI dừng không khớp P002; không DROP')
                if p['z']>=start['z']-self.cfg['motion']['xyz_tolerance']:
                    raise RuntimeError('Chưa có Z hạ dưới P002; không DROP')
            if completed and not laser_seen:
                laser_seen=True
                self._step('LASER_CONFIRMED')
            settled=settled+1 if completed else 0
            if settled>=self.cfg['motion']['settle_samples']:
                # Candidate only: automatic flow still waits for pressure and
                # parent JOB stopped before declaring MOVEUNTI_COMPLETE.
                return
            self._wait(self.cfg['motion']['poll_seconds'])
        raise RuntimeError('MOVEUNTI timeout: chưa đồng thời có laser ON và RUNNING OFF; không DROP')

    def _wait_vacuum_ready(self):
        self._step('VACUUM_ACTIVATING')
        deadline=time.monotonic()+self.cfg['vacuum'].get('job_timeout',10)
        stable=[0,0]
        while time.monotonic()<deadline:
            self._check_cancel()
            data=self.refresh()  # IOREAD 20050 includes pressure inputs 20051/20052.
            ready(data['flags'],allow_running=True)
            workspace(data['position'],self.cfg)
            if data.get('vacuum_input_contacts')!=[20051,20052]:
                raise RuntimeError('Pressure phải đọc đúng input 20051/20052; không DROP')
            values=data.get('vacuum_inputs',[])
            if len(values)!=2 or any(type(v) is not bool for v in values):
                raise RuntimeError('Pressure UNKNOWN/lỗi đọc I/O; không DROP')
            if data['laser'] is None:
                raise RuntimeError('Laser UNKNOWN khi chờ vacuum; không DROP')
            job=self.robot.current_job()
            if job not in ('MOVEUNTI',self.cfg['vacuum']['jobs']['2_on']):
                raise RuntimeError('JOB thay đổi khi chờ vacuum; không DROP')
            # Operator confirms H2ON is the final action before MOVEUNTI END.
            # A child H2ON or a resumed/running parent resets the stable samples.
            stopped=job=='MOVEUNTI' and data['flags']['running'] is False and data['laser'] is True
            if stopped:
                target=dict(zip(AXES,map(float,self.journal.state['pick_variable']['expected'][2:8])))
                p=data['position']
                if (not reached({**p,'z':target['z']},target,self.cfg) or
                        p['z']>=target['z']-self.cfg['motion']['xyz_tolerance']):
                    raise RuntimeError('Vị trí cuối MOVEUNTI không hợp lệ; không DROP')
            stable=[count+1 if stopped and value else 0 for count,value in zip(stable,values)]
            if max(stable)>=self.cfg['motion']['settle_samples']:
                self._step('MOVEUNTI_COMPLETE',stop_position=data['position'],
                           moveunti_completion=dict(criteria='laser_stopped_and_vacuum',laser=True,
                                                    running=False,vacuum_inputs=values))
                return
            self._wait(self.cfg['motion']['poll_seconds'])
        raise RuntimeError('Vacuum timeout: chưa có pressure 20051 hoặc 20052 ON ổn định và MOVEUNTI dừng; không DROP')

    def _run(self,stage):
        try:
            self._run_batch(stage)
        finally:
            self.vision.clear_next_pick()
            if self.prefetch['state']!='INVALID':
                self.prefetch['state']='IDLE'

    def _run_batch(self,stage):
        repeat = stage=='full_cycle' and self.cfg.get('workflow',{}).get('repeat_until_empty',False)
        self.journal.update(automatic_repeat=repeat,stop_reason=None)
        while True:
            self._run_once(stage)
            if not repeat or self.journal.state['state']!='COMPLETE' or self.journal.state.get('stop_reason'):
                return
            try:
                with self.lock,self.robot.lock:
                    self._check_cancel()
                    ready(self.robot.status())
                    if self.robot.laser() is not False:
                        raise RuntimeError('Laser ON/UNKNOWN trước lượt gắp tiếp theo')
                    self._step('SYSTEM_CHECK',cycle_id=uuid.uuid4().hex,target=None,error='',pending=None,
                               outcome_unknown=False,vacuum_job=None,manual_job=None,job_dispatch=None,
                               job_start_attempted=False,moveunti_completion=None,drop_release=None,
                               drop_target=None,drop_safe_pose=None,drop_plan=None,drop_attempt_cycle_id=None,
                               post_drop_target=None,stop_reason=None,drop_variable=None,pick_variable=None)
            except Exception as exc:
                with self.lock:
                    if self.journal.state['state']!='HOLD':
                        self.journal.update(state='FAULT',error=str(exc),outcome_unknown=False)
                return

    def _run_once(self,stage):
        try:
            if stage in ('full_cycle','xy_only','moveunti_only'):
                target = self._target()
                if target is None:
                    self._step('COMPLETE',stop_reason='no_boards',error='')
                    return
                self._prepare_pick_variable(target)
            if stage in ('full_cycle','moveunti_only'):
                if self.cfg.get('workflow',{}).get('end_after_moveunti_start',False):
                    self._start_moveunti()
                    last=self.journal.state['target']
                    self._step('JOB_DISPATCHED',job_dispatch=dict(name='MOVEUNTI',accepted=True,completion_verified=False),
                               last_target=last['xyz'] if last else self.journal.state.get('last_target'))
                    # Update display once; this is not a wait for JOB completion.
                    # A read failure cannot undo the acknowledged handoff.
                    try:
                        self.refresh()
                    except Exception:
                        pass
                    return
                self._job()
                self._wait_vacuum_ready()
            if stage=='vacuum_only':
                name=self.cfg['vacuum']['jobs']['2_on']
                self._step('VACUUM_ACTIVATING',vacuum_job=dict(name=name,status='running'))
                self._execute_hand_job(2,True,self.cfg['vacuum'])
                self._step('VACUUM_ACTIVATING',vacuum_job=dict(name=name,status='complete'))
            if stage=='full_cycle':
                self._prepare_drop_variable()
                with self.lock,self.robot.lock:
                    self._check_cancel()
                    data=self.refresh()
                    ready(data['flags'])
                    values=data.get('vacuum_inputs',[])
                    if len(values)!=2 or any(type(v) is not bool for v in values) or not any(values):
                        raise RuntimeError('Pressure OFF/UNKNOWN trước START DROP; không DROP')
                    self._command('START','DROP',lambda:self.robot.client.execute('START','DROP'))
                    self._arm_drop_prefetch()
                self._finish_drop()
                return
            last = self.journal.state['target']
            self._step('COMPLETE',last_target=last['xyz'] if last else self.journal.state.get('last_target'))
            self.refresh()
        except Exception as exc:
            self.vision.clear_next_pick(str(exc))
            self.prefetch.update(state='INVALID',reason=str(exc))
            motion_unresolved=bool(self.journal.state.get('job_start_attempted')) or self.journal.state['pending'] is not None
            with self.lock:
                if self.journal.state['state'] != 'HOLD':
                    values={}
                    hand=self.journal.state.get('vacuum_job')
                    if hand and hand['status']=='running':
                        values['vacuum_job']={**hand,'status':'unknown'}
                    self.journal.update(state='FAULT',error=str(exc),outcome_unknown=motion_unresolved,**values)
            # Request HOLD once; no assertion that this stops the robot if communication is lost.
            if motion_unresolved and not self.cancel.is_set():
                try:
                    self.robot.hold()
                except Exception as hold_error:
                    with self.lock:
                        self.journal.update(error=f'{exc}; HOLD chưa xác nhận: {hold_error}',outcome_unknown=True)
            logging.getLogger('cycle').exception('Cycle fault')

    def hold(self):
        self.cancel.set()  # Cancel before waiting for any in-flight transaction.
        self.vision.clear_next_pick()
        self.prefetch.update(state='INVALID',reason='HOLD')
        with self.lock:
            self.journal.update(state='HOLD',error='Software HOLD; không thay thế E-STOP',outcome_unknown=True)
        try:
            self.robot.hold()
            self.refresh()
        except Exception as exc:
            self.journal.update(error=f'HOLD chưa xác nhận; robot có thể vẫn chuyển động: {exc}')
            raise

    def acknowledge(self, inspected=False):
        with self.lock:
            if self.busy:
                raise RuntimeError('Chờ worker kết thúc')
            if self.journal.state['outcome_unknown'] and not inspected:
                raise RuntimeError('Cần xác nhận đã kiểm tra robot và kết quả lệnh chưa xác định')
            if self.journal.state['state'] not in ('COMPLETE','JOB_DISPATCHED','FAULT','HOLD'):
                raise RuntimeError('Không có trạng thái cần ACK')
            data = self.refresh()
            if data['flags']['running'] is not False:
                raise RuntimeError('Chưa xác minh robot dừng')
            # ACK does not release hardware HOLD, reset alarms, enable servo, or resume JOB.
            self.journal.update(state='IDLE',error='',pending=None,outcome_unknown=False,target=None)

    def _hold_release_state_ready(self):
        if self.busy or self.journal.state['state']!='IDLE':
            raise RuntimeError('ACK về IDLE và chờ worker dừng trước khi nhả HOLD')
        if self.journal.state['pending'] or self.journal.state['outcome_unknown']:
            raise RuntimeError('Kiểm tra và ACK lệnh chưa xác định trước khi nhả HOLD')

    def release_hold(self,confirmed=False):
        with self.lock,self.robot.lock:
            self._hold_release_state_ready()
            if confirmed is not True:
                raise ValueError('Cần xác nhận nhả HOLD có thể cho phép chuyển động đang treo tiếp tục')
            hold_release_ready(self.robot.status())
            self.journal.intent('HOLD','0')
            try:
                self.robot.release_hold()
                self.journal.accepted()
                self.journal.update(hold_release=dict(accepted=True))
                data=self.refresh()
                if data['flags'].get('hold_command') is not False:
                    raise RuntimeError('Đã gửi HOLD OFF nhưng chưa đọc được command HOLD OFF')
            except Exception as exc:
                self.journal.update(state='FAULT',error=str(exc),outcome_unknown=True)
                raise

    def _servo_state_ready(self):
        if self.stopping.is_set() or self.busy or self.journal.state['state']!='IDLE':
            raise RuntimeError('Cần IDLE và worker đã dừng trước khi điều khiển servo')
        if self.journal.state['pending'] or self.journal.state['outcome_unknown']:
            raise RuntimeError('Kiểm tra và ACK lệnh chưa xác định trước khi điều khiển servo')

    def servo_on(self):
        return self.set_servo(True)

    def servo_off(self):
        return self.set_servo(False)

    def set_servo(self,on):
        if type(on) is not bool:
            raise ValueError('Servo yêu cầu on boolean')
        operation_state='SERVO_ENABLING' if on else 'SERVO_DISABLING'
        check=servo_on_ready if on else servo_off_ready
        with self.lock,self.robot.lock:
            self._servo_state_ready()
            if not self.robot.connected:
                raise RuntimeError('Kết nối robot trước khi điều khiển servo')
            check(self.robot.status())
            self.journal.intent('SVON','1' if on else '0')
            self.journal.update(state=operation_state,error='',servo_target=on,
                                servo_enable=dict(accepted=False,verified=False))
        try:
            with self.lock,self.robot.lock:
                if self.journal.state['state']!=operation_state or self.stopping.is_set():
                    raise RuntimeError('Đã dừng thao tác điều khiển servo')
                # Recheck after the journal write, before sending the command.
                check(self.robot.status())
                self.robot.set_servo(on)
                self.journal.accepted()
                self.journal.update(servo_enable=dict(accepted=True,verified=False))
            deadline=time.monotonic()+self.cfg['timeout_seconds']
            while time.monotonic()<deadline:
                with self.lock,self.robot.lock:
                    if self.journal.state['state']!=operation_state or self.stopping.is_set():
                        raise RuntimeError('Đã dừng thao tác điều khiển servo')
                    data=self.refresh()
                    if self.journal.state['state']!=operation_state or self.stopping.is_set():
                        raise RuntimeError('Đã dừng thao tác điều khiển servo')
                    check(data['flags'],True)
                    if data['flags']['servo'] is on:
                        self.journal.update(state='IDLE',error='',
                                            servo_enable=dict(accepted=True,verified=True))
                        return
                self.stopping.wait(self.cfg['motion']['poll_seconds'])
            label='ON' if on else 'OFF'
            raise RuntimeError(f'SVON đã ACK nhưng chưa đọc được SERVO {label}; kiểm tra EXSVON/controller')
        except Exception as exc:
            with self.lock:
                if self.journal.state['state']!='HOLD':
                    self.journal.update(state='FAULT',error=str(exc),outcome_unknown=True)
            raise

    def vacuum(self, channel, on):
        with self.lock, self.robot.lock:
            if channel not in (1,2) or type(on) is not bool:
                raise ValueError('Cần channel 1/2 và on boolean')
            if self.busy or self.journal.state['state'] not in ('IDLE','COMPLETE'):
                raise RuntimeError('Vacuum manual bị khóa trong cycle/HOLD/FAULT')
            validate_mapping(self.robot.vacuum_config)
            manual_io_ready(self.robot.status())
            if self.robot.vacuum_config.get('method')=='job':
                previous=self.journal.state['state']
                self.cancel.clear()
                name=self.robot.vacuum_config['jobs'][f'{channel}_{"on" if on else "off"}']
                self.journal.update(state='VACUUM_ACTIVATING',error='',
                                    manual_job=dict(name=name,status='running'))
                self.thread=threading.Thread(target=self._manual_hand_job,args=(channel,on,previous),
                                             daemon=True,name='manual-hand')
                self.thread.start()
                return
            self.journal.intent('VACUUM',dict(channel=channel,on=on))
            try:
                self.robot.set_vacuum(channel,on)
                self.journal.accepted()
                self.refresh()
            except Exception as exc:
                self.journal.update(state='FAULT',error=str(exc),outcome_unknown=True)
                raise

    def _execute_hand_job(self,channel,on,config=None,require_laser=False,required_pose=None):
        mapping=self.robot.vacuum_config if config is None else config
        name=mapping['jobs'][f'{channel}_{"on" if on else "off"}']
        with self.lock,self.robot.lock:
            self._check_cancel()
            flags=self.robot.status()
            manual_io_ready(flags)
            if required_pose is not None:
                ready(flags)
                current,_=self.robot.pose()
                if not reached(current,required_pose,self.cfg):
                    raise RuntimeError('Robot không còn ở Z_DROP; không gọi H2OFF')
            if require_laser:
                ready(flags)
                if self.robot.current_job()!='MOVEUNTI' or self.robot.laser() is not True:
                    raise RuntimeError('Điều kiện MOVEUNTI/laser đã thay đổi trước H2ON; không gửi lệnh')
            self._command('START',name,lambda:self.robot.start_hand_job(channel,on,mapping))
        deadline=time.monotonic()+mapping.get('job_timeout',10)
        settled=0
        while time.monotonic()<deadline:
            self._check_cancel()
            with self.robot.lock:
                current=self.robot.current_job()
                flags=self.robot.status()
            manual_io_ready(flags,allow_running=True)
            if current!=name:
                raise RuntimeError(f'JOB hiện tại {current}, cần {name}; kiểm tra pendant')
            settled=settled+1 if flags['running'] is False else 0
            if settled>=2:
                self.refresh()
                return
            self._wait(self.cfg['motion']['poll_seconds'])
        raise RuntimeError(f'{name} timeout; chưa xác nhận JOB kết thúc')

    def _manual_hand_job(self,channel,on,previous):
        name=self.robot.vacuum_config['jobs'][f'{channel}_{"on" if on else "off"}']
        try:
            self._execute_hand_job(channel,on)
            self._step(previous,manual_job=dict(name=name,status='complete'))
        except Exception as exc:
            with self.lock:
                if self.journal.state['state']!='HOLD':
                    self.journal.update(state='FAULT',error=str(exc),outcome_unknown=True,
                                        manual_job=dict(name=name,status='unknown'))
            logging.getLogger('cycle').exception('HAND job fault')

    def configure(self, values, drop_values=None):
        with self.lock, self.vision.lock:
            if self.busy or self.journal.state['state'] != 'IDLE':
                raise RuntimeError('Chỉ sửa cấu hình khi IDLE')
            validated=validate_settings(values)
            if drop_values is not None:
                self._stack_edit_ready()
                candidate=self.stack.validate_values(drop_values)
                low,high=self.cfg['workspace']['z']
                if not low<=candidate['first_drop_z_mm']<=min(high,self.cfg['drop']['safe_z_mm']):
                    raise ValueError('Z thả đầu phải trong workspace và không cao hơn DROP SAFE')
                self.stack.configure_values(drop_values)
            self.cfg['settings'].update(validated)
            # Stored separately for MOCK/REAL, never silently transferred between modes.
            import json
            path = self.journal.path.parent/'operator_settings.json'
            temp = path.with_suffix('.tmp')
            temp.write_text(json.dumps(self.cfg['settings']),encoding='utf-8')
            temp.replace(path)
            self.journal.update(target=None)

    def close(self):
        self.cancel.set()
        self.stopping.set()
        # Shutdown does not claim the physical robot is stopped. Pending execution persists.
        if self.thread:
            self.thread.join(timeout=4)
            if self.thread.is_alive():
                raise RuntimeError('Cycle worker chưa dừng; giữ khóa process và journal')
        if self.poller:
            self.poller.join(timeout=4)
        self.vision.close()
        self.instance_lock.close()
