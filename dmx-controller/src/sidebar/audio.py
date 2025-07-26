# me - this DAT
# 
# channel - the Channel object which has changed
# sampleIndex - the index of the changed sample
# val - the numeric value of the changed sample
# prev - the previous sample value
# 
# Make sure the corresponding toggle is enabled in the CHOP Execute DAT.

sliders = [
    'slider1',
    'slider7',
    'slider20',
    'slider32',
    'slider37',
]

def onValueChange(channel, sampleIndex, val, prev):
    button_val = op('side_bar_out')['Audio'].eval()

    if button_val:
        for i in range(len(sliders)):
            op(f'GUI/{sliders[i]}/field1/string')[0, 0] = round(val)

    return
