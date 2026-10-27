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

    def execute(self, name, data=''):
        self.commands.append((name, data))
        if self.scenario == 'tcp_lost':
            raise OutcomeUnknown('TCP lost; robot may be moving')
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
            return 'MOVEUNTI,HOME' if self.available_job else 'HOME'
        if name == 'RJSEQ':
            return self.last_job+',1,1'
        if name == 'SAVEV':
            if data=='0,90':
                return str(self.b090)
            return ','.join(self.p002 if data=='4,2' else self.p009 if data=='4,9' else self.p010)
        if name == 'LOADV':
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
