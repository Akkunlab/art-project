from pyudmx import pyudmx
import time

cv = [0 for _ in range(512)]

print('Opening DMX controller...')
dev = pyudmx.uDMXDevice()
dev.open()
print(dev.Device)

# 内部状態
_pending = False
_last_frame = -1
_last_time  = 0.0
_MIN_INTERVAL = 1.0 / 40.0  # 上限

def _current_frame():
    try:
        return int(op('/').absTime.frame)
    except Exception:
        return -1

def send():
    global _pending
    _pending = True

def flush():
    global _pending, _last_frame, _last_time

    # 送信する値がない場合は何もしない
    if not _pending:
        return

    f = _current_frame()

    if f >= 0:
        if f == _last_frame:
            return

        dev.send_multi_value(1, cv)
        _last_frame = f
        _last_time = time.monotonic()
        _pending = False
        print('DMX signal sent!')
        return

    now = time.monotonic()

    if now - _last_time < _MIN_INTERVAL:
        return

    dev.send_multi_value(1, cv)
    _last_time = now
    _pending = False
    print('DMX signal sent!')

def close():
    global cv
    cv = [0 for _ in range(512)]

    try:
        dev.send_multi_value(1, cv)
    except Exception:
        pass

    dev.close()
    print('DMX controller closed!')
