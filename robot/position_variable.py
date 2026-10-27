"""HW0483202 3.2: Cartesian BASE pick P002 and P009-derived DROP P010."""
from decimal import Decimal
from core.settings import AXES
from .protocol import encode_command, ProtocolError


def parse_p010(raw,source='P010'):
    return parse_position(raw,10,source)


def parse_position(raw,index,source):
    fields=tuple(v.strip() for v in raw.split(','))
    # SAVEV reply has no variable type/index prefix; LOADV adds 4,index.
    try:
        encode_command('LOADV',','.join(('4',str(index))+fields))
    except ValueError as exc:
        raise ProtocolError(source+' cần dữ liệu Cartesian BASE hợp lệ: '+str(exc)) from exc
    return fields


def pick_position(target,reference):
    fields=('1','0')+tuple(f'{target[a]:.3f}' for a in AXES)+reference.fields[6:8]
    payload=','.join(('4','2')+fields)
    encode_command('LOADV',payload)
    return fields,payload


def verify_p002(expected,raw):
    expected=parse_position(','.join(expected),2,'P002 expected')
    actual=parse_position(raw,2,'P002')
    for index,(a,b) in enumerate(zip(actual,expected)):
        tolerance=Decimal('.001') if 2<=index<8 else Decimal('0')
        if abs(Decimal(a)-Decimal(b))>tolerance:
            raise ProtocolError('Đọc lại P002 không khớp; không START MOVEUNTI')


def with_z(fields,z):
    value=Decimal(str(z))
    if not value.is_finite() or value!=value.quantize(Decimal('.001')):
        raise ValueError('Z P010 cần số hữu hạn, tối đa 3 chữ số thập phân')
    result=fields[:4]+(format(value,'.3f'),)+fields[5:]
    payload=','.join(('4','10')+result)
    encode_command('LOADV',payload)
    return result,payload,dict(zip(AXES,map(float,result[2:8])))


def verify_p010(expected,raw):
    actual=parse_p010(raw)
    if any(Decimal(a)!=Decimal(b) for a,b in zip(actual,expected)):
        raise ProtocolError('Đọc lại P010 không khớp; không gọi DROP')
