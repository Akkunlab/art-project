from pyudmx import pyudmx

# 初期化
cv = [0 for v in range(0, 512)]

print('Opening DMX controller...')
dev = pyudmx.uDMXDevice()
dev.open()
print(dev.Device)

# DMX信号 送信
def send():
    dev.send_multi_value(1, cv)
    print('DMX signal sent!')

# 終了処理
def close():
    global cv
    cv = [0 for v in range(0, 512)]
    send()
    dev.close()
    print('DMX controller closed!')
