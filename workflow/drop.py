"""Persistent stack arithmetic, P009-derived P010, and compatible operator APIs."""
from copy import deepcopy
from decimal import Decimal
import json
import math
import os
import threading

from .safety_checks import ready,workspace
from robot.position_variable import parse_p010,with_z,verify_p010


class StackStore:
    def __init__(self,path,config):
        self.path=path
        self.lock=threading.RLock()
        self.data=dict(first_drop_z_mm=config.get('first_drop_z_mm',330),
                       board_thickness_mm=config.get('board_thickness_mm',12),
                       stacked_count=0,last_completed_drop_cycle_id=None,completed_drop_cycle_ids=[],last_picked_id=None)
        if path.exists():
            try:
                self.data.update(json.loads(path.read_text(encoding='utf-8')))
            except (OSError,ValueError) as exc:
                raise RuntimeError('Không đọc được bộ đếm DROP; không tự đặt lại số tấm') from exc
        self._validate(self.data)
        if not path.exists():
            self._save(self.data)

    @staticmethod
    def _validate(data):
        first,thickness=data['first_drop_z_mm'],data['board_thickness_mm']
        if any(type(v) not in (int,float) or not math.isfinite(v) for v in (first,thickness)):
            raise ValueError('Chiều cao/chiều dày phải là số hữu hạn')
        if not 0<thickness<=1000 or Decimal(str(thickness))!=Decimal(str(thickness)).quantize(Decimal('.001')):
            raise ValueError('Chiều dày cần >0, <=1000 mm, tối đa 3 số thập phân')
        if type(data['stacked_count']) is not int or data['stacked_count']<0:
            raise ValueError('Bộ đếm DROP không hợp lệ')
        if data.get('last_picked_id') is not None and (type(data['last_picked_id']) is not int or data['last_picked_id']<1):
            raise ValueError('ID gắp cuối không hợp lệ')
        ids=data['completed_drop_cycle_ids']
        if not isinstance(ids,list) or any(not isinstance(v,str) or not v for v in ids):
            raise ValueError('Lịch sử cycle DROP không hợp lệ')

    def _save(self,data):
        self.path.parent.mkdir(parents=True,exist_ok=True)
        temp=self.path.with_suffix('.tmp')
        with temp.open('w',encoding='utf-8') as stream:
            json.dump(data,stream,ensure_ascii=False,allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp,self.path)

    def _commit(self,data):
        self._validate(data)
        self._save(data)
        self.data=data

    def snapshot(self):
        with self.lock:
            result=deepcopy(self.data)
            result.pop('completed_drop_cycle_ids')
            result['next_board']=result['stacked_count']+1
            result['next_drop_z_mm']=float(Decimal(str(result['first_drop_z_mm']))+
                result['stacked_count']*Decimal(str(result['board_thickness_mm'])))
            return result

    def configure(self,thickness):
        self.configure_values({'board_thickness_mm':thickness})

    def validate_values(self,values):
        if not isinstance(values,dict) or set(values)-{'first_drop_z_mm','board_thickness_mm'}:
            raise ValueError('Chỉ cho phép Z thả đầu và độ dày ván')
        candidate={**self.data,**values}
        self._validate(candidate)
        first=candidate['first_drop_z_mm']
        if Decimal(str(first)).normalize().as_tuple().exponent < -3:
            raise ValueError('Z thả đầu tối đa 3 số thập phân')
        return candidate

    def configure_values(self,values):
        with self.lock:
            self._commit(self.validate_values(values))

    def reset(self):
        with self.lock:
            self._commit({**self.data,'stacked_count':0})

    def set_count(self,count):
        if type(count) is not int or not 0<=count<=9007199254740991:
            raise ValueError('Số tấm phải là số nguyên không âm hợp lệ')
        with self.lock:
            self._commit({**self.data,'stacked_count':count})

    def confirm(self,cycle_id,picked_id=None):
        with self.lock:
            if not isinstance(cycle_id,str) or not cycle_id:
                raise ValueError('Thiếu cycle ID')
            if cycle_id in self.data['completed_drop_cycle_ids']:
                return False
            self._commit({**self.data,'stacked_count':self.data['stacked_count']+1,
                          'last_picked_id':picked_id if picked_id is not None else self.data.get('last_picked_id'),
                          'last_completed_drop_cycle_id':cycle_id,
                          'completed_drop_cycle_ids':self.data['completed_drop_cycle_ids']+[cycle_id]})
            return True


class DropWorkflow:
    def _prepare_drop_variable(self):
        if self.cfg.get('drop',{}).get('position_variable')!=10:
            raise RuntimeError('Chỉ hỗ trợ biến DROP P010')
        with self.lock,self.robot.lock:
            self._check_cancel()
            ready(self.robot.status())
            height=self.stack.snapshot()
            before=parse_p010(self.robot.client.execute('SAVEV','4,9'),source='P009')
            expected,payload,target=with_z(before,height['next_drop_z_mm'])
            workspace(target,self.cfg)
            if target['z']>self.cfg['drop']['safe_z_mm']+self.cfg['motion']['xyz_tolerance']:
                raise RuntimeError('Z thả cao hơn DROP SAFE; không ghi P010 hoặc gọi DROP')
            self._step('WRITING_DROP_P010',drop_target=target,drop_plan=height,
                       drop_variable={'name':'P010','source':'P009','before':list(before),'expected':list(expected),'verified':False,
                                      'cycle_id':self.journal.state['cycle_id'],
                                      'connection_generation':self.robot.connection_generation})
            self._command('LOADV',payload,lambda:self.robot.client.execute('LOADV',payload))
            verify_p010(expected,self.robot.client.execute('SAVEV','4,10'))
            self._step('DROP_P010_WRITTEN',drop_variable={**self.journal.state['drop_variable'],'verified':True})

    def _can_confirm_drop(self):
        s=self.journal.state
        receipt=s.get('drop_release') or {}
        return (not self.busy and s['state'] in ('AWAIT_DROP_CONFIRMATION','FAULT')
                and not s['pending'] and receipt.get('job_completed') is True
                and receipt.get('cycle_id')==s['cycle_id'])

    def confirm_drop(self,cycle_id,confirmed=False):
        with self.lock:
            if confirmed is not True:
                raise ValueError('Cần xác nhận đã nhìn thấy ván nhả khỏi đầu hút')
            if not isinstance(cycle_id,str) or cycle_id!=self.journal.state['cycle_id']:
                raise RuntimeError('Cycle xác nhận không khớp')
            if self.journal.state['state']=='COMPLETE' and self.stack.snapshot()['last_completed_drop_cycle_id']==cycle_id:
                return
            if not self._can_confirm_drop():
                raise RuntimeError('Chưa xác nhận H2OFF hoàn thành; không tăng bộ đếm')
            if self.refresh()['flags']['running'] is not False:
                raise RuntimeError('Robot chưa dừng; không xác nhận thả')
            self.stack.confirm(cycle_id,picked_id=(self.journal.state.get('target') or {}).get('id'))
            self.journal.update(state='STACK_COUNT_UPDATED',error='',outcome_unknown=False,
                                drop_release={**self.journal.state['drop_release'],'board_release_confirmed':True})
            last=self.journal.state['target']
            self.journal.update(state='COMPLETE',last_target=last['xyz'] if last else self.journal.state.get('last_target'))

    def _stack_edit_ready(self):
        if self.busy or self.journal.state['state'] not in ('IDLE','COMPLETE'):
            raise RuntimeError('Chỉ sửa/reset bộ đếm khi cycle không hoạt động và không chờ xác nhận thả')
        if self.robot.connected and self.robot.status()['running'] is not False:
            raise RuntimeError('Robot đang RUNNING hoặc chưa xác định; không sửa/reset bộ đếm')

    def configure_drop(self,thickness):
        with self.lock:
            self._stack_edit_ready()
            self.stack.configure(thickness)
            self.journal.update()

    def reset_stack(self,confirmed=False):
        with self.lock:
            self._stack_edit_ready()
            if confirmed is not True:
                raise ValueError('Cần xác nhận RESET STACK')
            self.stack.reset()
            self.journal.update()

    def set_stack_count(self,count):
        with self.lock:
            self._stack_edit_ready()
            self.stack.set_count(count)
            self.journal.update()
