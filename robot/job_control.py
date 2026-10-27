"""HW0483202 §3.2.1 / §3.2.2. No JBI upload or modification."""
import re


def exists(client):
    raw = client.execute('RJDIR', '*')
    names = [name.strip() for name in re.split(r'[,\r\n]+',raw) if name.strip()]
    if any(not re.fullmatch(r'[A-Za-z0-9_\-]{1,32}', n) for n in names):
        raise RuntimeError('RJDIR framing/names chưa được xác minh')
    return 'MOVEUNTI' in names


def current(client):
    raw = client.execute('RJSEQ')
    values = raw.split(',')
    if len(values) != 3 or values[0] != 'MOVEUNTI' or not all(v.strip().isdigit() for v in values[1:]):
        raise RuntimeError('RJSEQ không xác nhận MOVEUNTI')
    return raw


def start(client):
    return client.execute('START', 'MOVEUNTI')


def stop_reason_proven(robot, observed):
    # RSTATS + digital IN1 alone cannot establish the cause of JOB termination.
    # REAL requires a site-specific, independently verified existing completion signal.
    # No invented register, ladder modification, or configurable "trust me" bypass.
    return robot.mock and observed['running_seen'] and observed['laser_edge']
