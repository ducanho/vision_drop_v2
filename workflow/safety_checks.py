import math
from core.settings import AXES, fingerprint


def ready(flags, allow_running=False):
    for key, expected in {'servo': True, 'remote': True, 'play': True, 'teach': False,
                          'alarm': False, 'error': False, 'hold': False}.items():
        if flags.get(key) is not expected:
            raise RuntimeError(f'Robot chưa sẵn sàng: {key}={flags.get(key)}')
    if not allow_running and flags.get('running') is not False:
        raise RuntimeError('Robot RUNNING hoặc UNKNOWN')


def workspace(pose, cfg):
    for axis in AXES:
        if not math.isfinite(pose[axis]):
            raise RuntimeError('Tọa độ không hữu hạn')
        if axis in cfg['workspace'] and not cfg['workspace'][axis][0] <= pose[axis] <= cfg['workspace'][axis][1]:
            raise RuntimeError(f'Ngoài workspace: {axis}')


def manual_io_ready(flags,allow_running=False):
    for key, expected in {'remote':True,'teach':False,'running':False,'alarm':False,'error':False,'hold':False}.items():
        if key=='running' and allow_running:
            continue
        if flags.get(key) is not expected:
            raise RuntimeError(f'Chưa thể điều khiển I/O: {key}={flags.get(key)}')


def hold_release_ready(flags):
    for key,expected in {'remote':True,'play':True,'teach':False,'running':False,
                         'alarm':False,'error':False,'hold_external':False,'hold_pendant':False,
                         'hold_command':True}.items():
        if flags.get(key) is not expected:
            raise RuntimeError(f'Chưa thể nhả HOLD: {key}={flags.get(key)}')


def servo_power_ready(flags,on,allow_target=False):
    for key,expected in {'remote':True,'play':True,'teach':False,'running':False,
                         'alarm':False,'error':False,'hold':False}.items():
        if flags.get(key) is not expected:
            raise RuntimeError(f'Chưa thể đổi trạng thái servo: {key}={flags.get(key)}')
    if flags.get('servo') is not (not on) and not (allow_target and flags.get('servo') is on):
        raise RuntimeError('Servo đã ở trạng thái yêu cầu hoặc chưa xác định')


def servo_on_ready(flags,allow_on=False):
    servo_power_ready(flags,True,allow_on)


def servo_off_ready(flags,allow_off=False):
    servo_power_ready(flags,False,allow_off)


def commissioning_hash(cfg):
    return fingerprint({k: v for k, v in cfg.items() if k not in ('commissioning', 'web')})


def operator_motion_workflow(cfg):
    flow=cfg.get('workflow',{})
    return flow.get('end_after_moveunti_start',False) or flow.get('z_completion')=='laser_on_and_stopped'


def real_gate(cfg, stage='full_cycle'):
    if stage == 'full_cycle':
        from .pickdrop import real_blockers
        issues=real_blockers(cfg)
        if issues:
            raise RuntimeError('REAL PICKDROP bị khóa: '+'; '.join(issues))
        return
    # Operator confirmed the existing MOVL path and requested removal of the
    # static commissioning/expected-tool gates for the handoff workflow.
    # Live readiness, workspace, pose metadata and JOB existence stay checked.
    if operator_motion_workflow(cfg) and stage in ('xy_only','moveunti_only'):
        if stage!='xy_only' and cfg['job']['name']!='MOVEUNTI':
            raise RuntimeError('Luồng bàn giao yêu cầu đúng JOB MOVEUNTI')
        return
    c = cfg['commissioning']
    required = ['read_only']
    if stage == 'xy_only':
        required += ['path_verified', 'orientation_verified']
        if cfg['motion']['expected_tool'] is None or cfg['motion']['expected_posture'] is None:
            raise RuntimeError('Chưa xác minh TOOL/posture')
    if any(c.get(k) is not True for k in required) or not c['evidence'] or c['config_sha256'] != commissioning_hash(cfg):
        raise RuntimeError('REAL bị khóa: thiếu nghiệm thu hoặc cấu hình đã thay đổi; xem checklist')
    if stage == 'moveunti_only':
        j = cfg['job']
        if not (j['verified'] and j['start_verified'] and cfg['laser']['verified']
                and isinstance(j['min_z'], (float, int)) and isinstance(j['max_descent_mm'], (float, int))
                and isinstance(j['speed_mm_s'], (float, int)) and j['speed_mm_s'] > 0 and j['max_descent_mm'] > 0):
            raise RuntimeError('MOVEUNTI: thiếu xác minh JOB/laser/hành trình/tốc độ')
    if stage == 'vacuum_only':
        from robot.vacuum import validate_mapping
        validate_mapping(cfg['vacuum'])


def reached(current, target, cfg):
    return all(abs((current[a]-target[a]+180) % 360-180) <= cfg['motion']['angle_tolerance']
               if a.startswith('r') else abs(current[a]-target[a]) <= cfg['motion']['xyz_tolerance'] for a in AXES)
