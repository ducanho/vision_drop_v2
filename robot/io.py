"""Digital inputs only. A failed/malformed read is UNKNOWN, never OFF."""
import re


def read_byte(client, start):
    raw = client.execute('IOREAD', f'{start},8')
    if not re.fullmatch(r'\d{1,3}', raw) or not 0 <= int(raw) <= 255:
        raise RuntimeError('IOREAD: phản hồi không hợp lệ')
    return int(raw)


def laser_sample(client, cfg):
    if cfg['start'] != 20050 or not 0 <= cfg['bit'] <= 7:
        raise RuntimeError('Laser mapping không hợp lệ')
    raw = read_byte(client, cfg['start'])
    return bool(raw & (1 << cfg['bit'])) == cfg['active_high'], raw


def laser(client, cfg):
    # Read-only telemetry is available before commissioning is marked verified.
    # Workflow commissioning still controls use of this signal for JOB execution.
    return laser_sample(client, cfg)[0]
