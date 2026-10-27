"""Verified network-input byte owned exclusively by this application."""
import re
from .io import read_byte


def validate_mapping(cfg):
    if cfg.get('method')=='job':
        names=cfg.get('jobs',{})
        if (cfg.get('verified') is not True or set(names)!={'1_on','1_off','2_on','2_off'}
                or len(set(names.values()))!=4 or any(not isinstance(n,str) or not re.fullmatch(r'[A-Za-z0-9_\-]{1,32}',n)
                or n=='MOVEUNTI' for n in names.values())):
            raise RuntimeError('Chưa xác nhận đúng bốn JOB HAND ON/OFF trên pendant')
        return
    if cfg.get('start') is None or any(b is None for b in cfg.get('bits',[])):
        raise RuntimeError('Đã xác định OUTPUT 30050/30051; còn thiếu network input 22xxx liên kết tới hai relay để gửi ON/OFF qua TCP.')
    if (not cfg['verified'] or cfg['mode'] not in ('independent', 'complementary')
            or cfg['owned_byte'] is not True or type(cfg['start']) is not int
            or not 22010 <= cfg['start'] <= 23280 or cfg['start'] % 10 != 0
            or len(cfg['bits']) != 2 or any(type(b) is not int or not 0 <= b <= 7 for b in cfg['bits'])
            or cfg['bits'][0] == cfg['bits'][1] or len(cfg['active_high']) != 2
            or any(type(v) is not bool for v in cfg['active_high'])):
        raise RuntimeError('HAND REAL bị khóa: chưa xác minh mapping, cực tính, loại HAND và quyền sở hữu byte')


def monitor_inputs(raw_byte, byte_start, contacts):
    if not contacts:
        return [None,None]
    if len(contacts)!=2 or any(type(a) is not int or a//10!=byte_start//10 or not 0<=a%10<=7 for a in contacts):
        raise ValueError('Địa chỉ I/O vacuum không thuộc byte đang đọc')
    if raw_byte is None:
        return [None,None]
    return [bool(raw_byte & (1<<(a%10))) for a in contacts]


def read_outputs(client,cfg):
    contacts=cfg.get('output_contacts',[])
    if not contacts:
        return None,None
    start=contacts[0]//10*10
    # Validate both contact addresses before any I/O transaction.
    monitor_inputs(None,start,contacts)
    raw=read_byte(client,start)
    return monitor_inputs(raw,start,contacts),raw


def write(client, cfg, channel, on):
    validate_mapping(cfg)
    if cfg.get('method')=='job':
        raise RuntimeError('HAND JOB phải được theo dõi qua workflow, không dùng IOWRITE')
    byte = read_byte(client, cfg['start'])
    values = [bool(byte & (1 << b)) == high
              for b, high in zip(cfg['bits'], cfg['active_high'])]
    values[channel-1] = on
    if cfg['mode'] == 'complementary':
        values[2-channel] = not on
    # Write both bits in ONE byte transaction, preserving unrelated bits.
    for state, bit, high in zip(values, cfg['bits'], cfg['active_high']):
        byte = byte | (1 << bit) if state == high else byte & ~(1 << bit)
    client.execute('IOWRITE', f'{cfg["start"]},8,{byte}')
    result = read_byte(client, cfg['start'])
    if result != byte:
        raise RuntimeError('HAND readback không khớp; kết quả chưa xác định')
    outputs,_=read_outputs(client,cfg)
    if outputs is not None:
        if outputs[channel-1] is not on:
            raise RuntimeError('Network input đã đổi nhưng OUTPUT relay chưa đúng yêu cầu; kiểm tra mapping, không tự gửi lại')
        return outputs
    return values
