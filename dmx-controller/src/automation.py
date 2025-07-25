# me - this DAT
# channel - the Channel object which has changed
# sampleIndex - the index of the changed sample
# val - the numeric value of the changed sample
# prev - the previous sample value
# Make sure the corresponding toggle is enabled in the CHOP Execute DAT.

sliders = [
    'slider1',
    'slider7',
    'slider20',
    'slider32',
    'slider37',
]

def onValueChange(channel, sampleIndex, val, prev):
    button_val = op('/project1/side_bar/null1')['v5']

    if button_val:
        for i in range(len(sliders)):
            op(f'/project1/GUI/{sliders[i]}/field1/string')[0, 0] = round(val)

    return
