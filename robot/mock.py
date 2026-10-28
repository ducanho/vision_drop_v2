"""Deterministic controller simulator. Never opens a network socket."""
import time
from core.settings import AXES
from .protocol import OutcomeUnknown


class MockClient:
    def __init__(self,hand_jobs=()):
        self.pose = [700., 0., 500., 180., 0., 0.]
        self.tool, self.posture = 0, 0
        self.running = self.holding = self.alarm = self.laser = False
        self.servo=True
        self.available_job = True
        self.available_home = True
        self.scenario = ''
        self.commands = []
        self.deadline = 0
        self.activity = None
        self.byte = 0
        self.pressure_inputs=[False,False]
        self.hand_jobs=set(hand_jobs)
        self.last_job='MOVEUNTI'
        self.drop_pose=[700.,-800.,600.,180.,0.,0.]
        self.p010=['1','0','700','-800','600','180','0','0','0','0']
        self.p009=self.p010[:]
        self.p002=['1','0']+list(map(str,self.pose))+['0','0']
        self.drop_uses_p010=False
        self.b090=0
        self.drop_checkpoint=0
        self.b091=0
        self.p011=['1','0','0','0','200','0','0','0','0','0']
        self.hand2_on=False
        self.i000=3500

    def execute(self, name, data=''):
        self.commands.append((name, data))
        if self.scenario == 'tcp_lost':
            raise OutcomeUnknown('TCP lost; robot may be moving')
        if self.activity=='pickdrop' and not self.holding:
            if self.scenario=='pickdrop_disconnect':
                raise OutcomeUnknown('PICKDROP telemetry lost')
            elapsed=time.monotonic()-self.pickdrop_started
            # Abstract internal laser search and vacuum OR. These are simulated
            # sensors, never PC commands, and do not validate the malformed JBI.
            if elapsed>=.05:
                self.laser=self.scenario not in ('laser_timeout','unproven_stop')
                self.hand2_on=self.laser
            if elapsed>=.1 and self.laser:
                self.pressure_inputs=([False,False] if self.scenario=='vacuum_timeout' else
                                      [False,True] if self.scenario=='vacuum_sensor_2' else [True,False])
            fault=(elapsed>=.15 and (not self.laser or not any(self.pressure_inputs)))
            if time.monotonic() >= self.drop_checkpoint and not fault:
                self.b090=1
            if fault or time.monotonic() >= self.deadline:
                if self.scenario in ('pickdrop_fault','laser_timeout','unproven_stop','vacuum_timeout'):
                    self.b091=2
                    self.running=False  # PAUSE at fault; no release or lift.
                    self.activity=None
                elif self.scenario not in ('pickdrop_timeout','move_timeout'):
                    self.pose=list(map(float,self.p010[2:8]))
                    for i in range(3):
                        self.pose[i]+=float(self.p011[2+i])
                    if self.scenario=='pickdrop_bad_pose':
                        self.pose[2]+=20
                    self.b091=1
                    self.hand2_on=False
                    self.pressure_inputs=[False,False]
                    self.laser=False
                    self.running=False
                    self.activity=None
        if self.activity=='drop' and time.monotonic() >= self.drop_checkpoint:
            self.b090=1
        if self.activity and time.monotonic() >= self.deadline and not self.holding:
            if self.activity=='approach' and self.scenario!='move_timeout':
                self.pose=list(map(float,self.p002[2:8]))
                self.activity,self.deadline='pick_at_target',time.monotonic()+.15
            elif self.activity=='pick_at_target':
                self.activity,self.deadline='job',time.monotonic()+.35
            elif self.activity == 'move' and self.scenario != 'move_timeout':
                self.pose = self.target[:]
                self.running = False
                self.activity = None
            elif self.activity == 'job' and self.scenario != 'laser_timeout':
                self.pose[2] -= 30
                self.laser = self.scenario != 'unproven_stop'
                self.running = False
                self.activity = None
            elif self.activity=='hand':
                self.running=False
                self.activity=None
            elif self.activity=='home' and self.scenario!='home_timeout':
                self.pose=[530.,270.,0.,180.,0.,90.]
                self.running=False
                self.activity=None
            elif self.activity=='drop':
                self.pose=list(map(float,self.p010[2:8])) if self.drop_uses_p010 else self.drop_pose[:]
                self.laser=False
                self.running=False
                self.activity=None
        if name == 'RSTATS':
            return f'{194 + (8 if self.running else 0)},{(64 if self.servo else 0) + (8 if self.holding else 0) + (16 if self.alarm else 0)}'
        if name == 'RPOSC':
            return ','.join(map(str, self.pose + [self.posture, self.tool] + [0]*6))
        if name == 'RPOSJ':
            return ','.join(['0']*6)
        if name == 'RJDIR':
            return ','.join((['MOVEUNTI','PICKDROP'] if self.available_job else [])+
                            (['HOME'] if self.available_home else []))
        if name == 'RJSEQ':
            return self.last_job+',1,1'
        if name == 'SAVEV':
            if data=='1,0':
                return str(self.i000)
            if data=='0,90':
                return str(self.b090)
            if data=='0,91':
                return str(self.b091)
            if data=='4,11':
                return ','.join(self.p011)
            return ','.join(self.p002 if data=='4,2' else self.p009 if data=='4,9' else self.p010)
        if name == 'LOADV':
            if data.startswith('1,0,'):
                value=int(data.split(',')[2])
                if not 0<=value<=5000:
                    raise ValueError('I000 outside speed range')
                self.i000=value
                if self.scenario=='lost_i000_reply':
                    raise OutcomeUnknown('LOADV I000 response lost')
                return '0000'
            if data.startswith('4,2,'):
                self.p002=data.split(',')[2:]
            else:
                self.p010=data.split(',')[2:]
                self.drop_uses_p010=True
            return '0000'
        if name == 'IOREAD':
            return str(int(self.laser or self.scenario == 'early_laser') | (int(self.pressure_inputs[0])<<1) | (int(self.pressure_inputs[1])<<2)) if data.startswith('20050,') else str(self.byte)
        if name == 'IOWRITE':
            self.byte = int(data.split(',')[2])
            return '0000'
        if name == 'MOVL':
            self.target = list(map(float, data.split(',')[3:9]))
            self.activity, self.running, self.deadline = 'move', True, time.monotonic()+.25
            if self.scenario == 'lost_movl_reply':
                raise OutcomeUnknown('MOVL response lost')
            return '0000'
        if name == 'START':
            if data=='HOME':
                if not self.available_home:
                    raise RuntimeError('JOB missing')
                self.last_job=data
                self.activity,self.running,self.deadline='home',True,time.monotonic()+.25
                if self.scenario=='lost_home_reply':
                    raise OutcomeUnknown('HOME response lost')
                return '0000'
            if data=='PICKDROP':
                if not self.available_job:
                    raise RuntimeError('JOB missing')
                self.b090=self.b091=0
                self.pickdrop_started=time.monotonic()
                self.last_job=data
                self.activity,self.running,self.deadline='pickdrop',True,time.monotonic()+.65
                self.drop_checkpoint=time.monotonic()+.2
                if self.scenario=='lost_pickdrop_reply':
                    raise OutcomeUnknown('PICKDROP response lost')
                return '0000'
            if data=='DROP' and data in self.hand_jobs:
                self.b090=0
                self.drop_checkpoint=time.monotonic()+.06
                self.last_job=data
                self.activity,self.running,self.deadline='drop',True,time.monotonic()+.15
                return '0000'
            if data in self.hand_jobs:
                self.last_job=data
                self.activity,self.running,self.deadline='hand',True,time.monotonic()+.15
                return '0000'
            if data != 'MOVEUNTI' or not self.available_job:
                raise RuntimeError('JOB missing')
            self.activity, self.running, self.deadline = 'approach', True, time.monotonic()+.15
            self.last_job=data
            return '0000'
        if name == 'HOLD':
            self.holding=data=='1'
            if self.holding:
                self.running=False
            return '0000'
        if name == 'SVON' and data in ('0','1'):
            self.servo=data=='1'
            return '0000'
        raise ValueError('Unsupported mock command')
