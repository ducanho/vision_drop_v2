"""Single controller JOB lifecycle. No PC motion or HAND commands."""
import re
import time
import hashlib

from core.settings import AXES, owned, load_config
from robot import job_control
from robot.position_variable import parse_p010, verify_p002, verify_p010
from .safety_checks import ready, reached, workspace


def real_blockers(cfg=None):
    """Validate the operator-confirmed JOB and the supported final IMOV frame."""
    contract=(cfg if cfg is not None else load_config()).get('pickdrop',{})
    path = owned('jobs/PICKDROP.JBI')
    issues = []
    if not path.is_file():
        issues.append('thiếu jobs/PICKDROP.JBI')
    else:
        source = path.read_text(encoding='utf-8-sig')
        if contract.get('job_sha256') != hashlib.sha256(path.read_bytes()).hexdigest():
            issues.append('bản PICKDROP.JBI thay đổi sau xác nhận của người vận hành')
        labels = set(re.findall(r'^\*(\w+)\s*$', source, re.M))
        for label in sorted(set(re.findall(r'JUMP \*(\w+)', source)) - labels):
            issues.append('JUMP *'+label+' không có nhãn *'+label)
        for line in ('//NAME PICKDROP', 'IMOV P008 V=I000 PL=0 DEC=50',
                     'IMOV P011 V=I000 PL=0', 'SET B090 0', 'SET B091 0',
                     'SET B090 1', 'SET B091 1', 'SET B091 2'):
            if line not in source.splitlines():
                issues.append('chưa đối chiếu hợp đồng: '+line)
    if contract.get('operator_confirmed') is not True:
        issues.append('chưa xác nhận JOB PICKDROP trên NX100')
    if contract.get('p011_frame') != 'BASE':
        issues.append('chưa xác nhận IMOV P011 là dịch chuyển theo BASE; cần hướng dịch chuyển thực tế')
    return issues


class PickDropWorkflow:
    def _start_pickdrop(self):
        self._step('START_PICKDROP')
        with self.lock, self.robot.lock:
            self._check_cancel()
            if self.journal.state.get('job_start_attempted'):
                raise RuntimeError('PICKDROP START đã được thử cho cycle này; không gửi lại')
            if self.journal.state['pending'] or self.journal.state['outcome_unknown']:
                raise RuntimeError('PICKDROP outcome unknown/pending; không START')
            if not self.mock:
                from .safety_checks import real_gate
                real_gate(self.cfg, 'full_cycle')
            ready(self.robot.status())
            self._check_pick_prefetch()
            cycle_id = self.journal.state['cycle_id']
            generation = self.robot.connection_generation
            for key in ('pick_variable', 'drop_variable'):
                receipt = self.journal.state.get(key) or {}
                if (not self.robot.connected or not receipt.get('verified') or
                        receipt.get('cycle_id') != cycle_id or
                        receipt.get('connection_generation') != generation):
                    raise RuntimeError('Chưa xác minh '+key+' cho cycle/kết nối hiện tại')
            pick = self.journal.state['pick_variable']
            drop = self.journal.state['drop_variable']
            verify_p002(pick['expected'], self.robot.client.execute('SAVEV', '4,2'))
            verify_p010(drop['expected'], self.robot.client.execute('SAVEV', '4,10'))
            pose, reference = self.robot.pose()
            workspace(pose, self.cfg)
            if list(reference.fields[6:]) != pick['metadata']:
                raise RuntimeError('Tool/posture/trục ngoài thay đổi sau ghi P002')
            if self.robot.laser() is not False:
                raise RuntimeError('Laser ON/UNKNOWN trước START PICKDROP')
            if not job_control.exists(self.robot.client, 'PICKDROP'):
                raise RuntimeError('JOB PICKDROP không tồn tại')
            # REAL uses this adapter only after the operator confirms BASE.
            # Read the live vector; never hardcode a Z+200 lift or emit PC motion.
            increment = parse_p010(self.robot.client.execute('SAVEV', '4,11'), source='P011')
            if any(float(v) != 0 for v in increment[5:8]):
                raise RuntimeError('P011 chỉ hỗ trợ translation BASE đã xác nhận; không suy đoán rotation')
            target = dict(self.journal.state['drop_target'])
            for axis, delta in zip(AXES[:3], increment[2:5]):
                target[axis] += float(delta)
            workspace(target, self.cfg)
            self.journal.update(pickdrop=dict(cycle_id=cycle_id, connection_generation=generation,
                                accepted=False, reset_confirmed=False, b091=None,
                                completed=False, final_target=target), job_start_attempted=True)
            ready(self.robot.status())
            self._check_pick_prefetch()
            self._command('START', 'PICKDROP', lambda: job_control.start(self.robot.client, 'PICKDROP'))
            self.journal.update(pickdrop={**self.journal.state['pickdrop'], 'accepted':True})
            if self.vision.prefetch_status().get('state') in ('PREPARED', 'VALIDATING'):
                self.vision.consume_next_pick()
                self._prefetch_event('NEXT_PICKDROP_SENT')
            # First post-ACK read, before any wait. Pre-START zero is not proof.
            self._arm_drop_prefetch()
            self._sample_pickdrop_result()

    def _sample_pickdrop_result(self):
        receipt = self.journal.state['pickdrop']
        if (receipt['cycle_id'] != self.journal.state['cycle_id'] or
                receipt['connection_generation'] != self.robot.connection_generation or
                not self.robot.connected):
            raise RuntimeError('PICKDROP cycle/connection changed')
        if self.robot.current_job() != 'PICKDROP':
            raise RuntimeError('RJSEQ không xác nhận PICKDROP của cycle hiện tại')
        value = self.robot.read_b091()
        if receipt['b091'] == 1 and value == 0:
            raise RuntimeError('B091 reset lại sau success checkpoint; cycle outcome unknown')
        receipt = {**receipt, 'b091':value,
                   'reset_confirmed':receipt['reset_confirmed'] or value == 0}
        self.journal.update(pickdrop=receipt)
        if value == 2:
            raise RuntimeError('PICKDROP FAULT B091=2: kiểm tra thủ công laser/vacuum/JOB; không tự phục hồi')
        if value not in (0, 1):
            raise RuntimeError('B091 UNKNOWN/invalid: '+str(value))
        if value == 1 and not receipt['reset_confirmed']:
            raise RuntimeError('B091=1 chưa quan sát reset trong cycle hiện tại')
        return value

    def _wait_pickdrop_complete(self):
        self._step('PICKDROP_RUNNING')
        deadline = time.monotonic()+self.cfg['drop'].get('job_timeout',120)
        settled = 0
        while time.monotonic() < deadline:
            self._check_cancel()
            with self.lock, self.robot.lock:
                self._check_cancel()
                data = self.refresh()
                if data.get('read_errors'):
                    raise RuntimeError('PICKDROP telemetry UNKNOWN: '+str(data['read_errors']))
                ready(data['flags'], allow_running=True)
                workspace(data['position'], self.cfg)
                value = self._sample_pickdrop_result()
                self._poll_drop_prefetch()
                receipt = self.journal.state['pickdrop']
                valid = (receipt['accepted'] and receipt['reset_confirmed'] and value == 1 and
                         data['flags']['running'] is False and
                         reached(data['position'], receipt['final_target'], self.cfg) and
                         self.journal.state['pending'] is None and not self.journal.state['outcome_unknown'])
                settled = settled+1 if valid else 0
                if settled >= self.cfg['motion']['settle_samples']:
                    self._step('PICKDROP_COMPLETE', pickdrop={**receipt, 'completed':True})
                    return
            self._wait(self.cfg['motion']['poll_seconds'])
        raise RuntimeError('PICKDROP timeout: B091/JOB/RUNNING/vị trí cuối chưa được xác minh')

    def _confirm_pickdrop(self):
        with self.lock, self.robot.lock:
            self._check_cancel()
            receipt = self.journal.state['pickdrop']
            if not receipt['completed'] or self.journal.state['pending'] or self.journal.state['outcome_unknown']:
                raise RuntimeError('PICKDROP chưa hoàn thành được xác minh')
            # Recheck immediately before durable count; no success from cached B091.
            data = self.refresh()
            ready(data['flags'])
            if (data.get('read_errors') or self._sample_pickdrop_result() != 1 or
                    not reached(data['position'], receipt['final_target'], self.cfg)):
                raise RuntimeError('PICKDROP completion changed before count')
            if self.prefetch['state'] == 'WAITING_B090':
                self.prefetch.update(state='INVALID', reason='B090 reset/trigger not proven before PICKDROP completion')
                self.vision.clear_next_pick(self.prefetch['reason'])
            cycle_id = self.journal.state['cycle_id']
            self._prefetch_event('PICKDROP_COMPLETED')
            self.stack.confirm(cycle_id, picked_id=(self.journal.state.get('target') or {}).get('id'))
            self._step('STACK_COUNT_UPDATED', drop_release=dict(cycle_id=cycle_id, job='PICKDROP',
                       internal_h2off=True, job_completed=True, board_release_confirmed=False,
                       counted=True, count_basis='pickdrop_completed'), post_drop_target=receipt['final_target'])
            last = self.journal.state.get('target')
            self._step('COMPLETE', last_target=last['xyz'] if last else self.journal.state.get('last_target'))
