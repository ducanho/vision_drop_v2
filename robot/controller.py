"""Every transaction is serialized; monitoring never holds the lock while sleeping."""
import re
import threading
from core.settings import AXES
from .protocol import Client
from .mock import MockClient
from .status import Status
from .cartesian import read_pose, build_move
from . import io, job_control, vacuum


class Controller:
    def __init__(self, config, mock):
        self.config, self.mock = config, mock
        self.lock = threading.RLock()
        self.connected = False
        self.connection_generation = 0
        allowed_jobs=tuple(config['vacuum'].get('jobs',{}).values()) if config['vacuum'].get('method')=='job' else ()
        allowed_jobs+=('DROP','PICKDROP','HOME')
        self.client = MockClient(allowed_jobs) if mock else Client(config['robot_ip'], config['robot_port'], config['timeout_seconds'],allowed_start_jobs=allowed_jobs)
        self.vacuum = [None, None]
        self.laser_raw_byte = None
        self.vacuum_raw_byte = None
        self.vacuum_config = dict(verified=True, mode='independent', start=22010, bits=[0,1],
                                  active_high=[True,True], owned_byte=True) if mock else config['vacuum']
        self.laser_config = dict(verified=True, start=20050, bit=0, active_high=True) if mock else config['laser']

    def connect(self):
        with self.lock:
            self.status()
            self.pose()
            self.client.execute('RPOSJ')
            self.connected = True
            self.connection_generation += 1

    def status(self):
        with self.lock:
            raw = self.client.execute('RSTATS')
            if not re.fullmatch(r'\d+,\d+', raw):
                raise RuntimeError('RSTATS malformed')
            s = Status(*map(int, raw.split(',')))
            return dict(servo=s.is_servo_on(), remote=s.is_command_remote(), play=s.is_play(),
                        teach=s.is_teach(), running=s.is_running(), alarm=s.is_alarm_occurring(),
                        error=s.is_error_occurring(), hold=s.is_command_hold() or s.is_external_hold() or s.is_programming_pendant_hold(),
                        hold_command=s.is_command_hold(),hold_external=s.is_external_hold(),hold_pendant=s.is_programming_pendant_hold())

    def pose(self):
        with self.lock:
            pose = read_pose(self.client)
            return dict(zip(AXES, map(float, pose.fields[:6]))), pose

    def read_b090(self):
        """Uncached read, serialized with every controller transaction."""
        return self._read_byte(90)

    def read_b091(self):
        return self._read_byte(91)

    def _read_byte(self, index):
        with self.lock:
            raw = self.client.execute('SAVEV', f'0,{index}').strip()
            if not re.fullmatch(r'[0-9]{1,3}', raw) or not 0 <= int(raw) <= 255:
                raise RuntimeError(f'SAVEV B{index:03d} malformed/out of range: ' + raw)
            return int(raw)

    def laser(self):
        with self.lock:
            try:
                value, self.laser_raw_byte = io.laser_sample(self.client, self.laser_config)
                return value
            except Exception:
                self.laser_raw_byte = None
                raise

    def read_i000(self):
        """HW0483202 SAVEV format 1: type 1, variable 0; signed Integer reply."""
        with self.lock:
            raw=self.client.execute('SAVEV','1,0').strip()
            if not re.fullmatch(r'[+-]?\d{1,5}',raw) or not -32768<=int(raw)<=32767:
                raise RuntimeError('SAVEV I000 malformed/out of Integer range: '+raw)
            return int(raw)

    def write_i000(self,value):
        if type(value) is not int or not 0<=value<=5000:
            raise ValueError('SPEED I000 cần số nguyên từ 0 đến 5000')
        with self.lock:
            self.client.execute('LOADV',f'1,0,{value}')

    def move(self, target, speed, reference):
        with self.lock:
            self.client.execute('MOVL', build_move('MOVL', str(speed), [f'{target[a]:.3f}' for a in AXES], reference))

    def hold(self):
        with self.lock:
            self.client.execute('HOLD', '1')

    def release_hold(self):
        with self.lock:
            self.client.execute('HOLD','0')

    def set_servo(self,on):
        if type(on) is not bool:
            raise ValueError('Servo yêu cầu on boolean')
        with self.lock:
            self.client.execute('SVON','1' if on else '0')

    def set_vacuum(self, channel, on):
        with self.lock:
            try:
                self.vacuum = vacuum.write(self.client, self.vacuum_config, channel, on)
            except Exception:
                self.vacuum = [None,None]
                raise

    def start_hand_job(self,channel,on,config=None):
        with self.lock:
            mapping=self.vacuum_config if config is None else config
            vacuum.validate_mapping(mapping)
            name=mapping['jobs'][f'{channel}_{"on" if on else "off"}']
            self.client.execute('START',name)

    def current_job(self):
        with self.lock:
            raw=self.client.execute('RJSEQ')
            parts=[p.strip() for p in raw.split(',')]
            if len(parts)!=3 or not all(p.isdigit() for p in parts[1:]):
                raise RuntimeError('RJSEQ không hợp lệ: '+raw)
            return parts[0]

    def observe(self):
        with self.lock:
            flags = {k:None for k in ('servo','remote','play','teach','running','alarm','error','hold')}
            pose, laser = None, None
            errors = {}
            received = False
            try:
                flags = self.status()
                received = True
            except Exception as exc:
                errors['status'] = str(exc)
            try:
                pose, _ = self.pose()
                received = True
            except Exception as exc:
                errors['position'] = str(exc)
            try:
                laser = self.laser()
                received = True
            except Exception as exc:
                message = str(exc)
                if '(2100)' in message:
                    message = 'IOREAD 2100: chưa đặt command REMOTE trên pendant. ' + message
                errors['laser'] = message
            if self.vacuum_config.get('output_contacts'):
                try:
                    self.vacuum,self.vacuum_raw_byte=vacuum.read_outputs(self.client,self.vacuum_config)
                    received=True
                except Exception as exc:
                    self.vacuum,self.vacuum_raw_byte=[None,None],None
                    errors['vacuum']=str(exc)
            if not received:
                self.connected = False
                raise RuntimeError('; '.join(errors.values()))
            self.connected = True
            input_contacts=self.config['vacuum'].get('input_contacts',[])
            vacuum_inputs=vacuum.monitor_inputs(self.laser_raw_byte,self.laser_config['start'],input_contacts)
            speed_i000,speed_error=None,''
            try:
                speed_i000=self.read_i000()
            except Exception as exc:
                # Speed display failure must not change existing motion telemetry guards.
                speed_error=str(exc)
            return dict(flags=flags, position=pose, laser=laser, laser_raw_byte=self.laser_raw_byte,
                        speed_i000=speed_i000,speed_error=speed_error,
                        laser_mapping_verified=self.laser_config['verified'], read_errors=errors,
                        vacuum_inputs=vacuum_inputs,vacuum_input_contacts=input_contacts,
                        vacuum_output_contacts=self.vacuum_config.get('output_contacts',[]),vacuum_raw_byte=self.vacuum_raw_byte,
                        error='; '.join(errors.values()), vacuum=self.vacuum[:], connected=True)
