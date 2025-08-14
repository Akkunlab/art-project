# me - this DAT
# 
# channel - the Channel object which has changed
# sampleIndex - the index of the changed sample
# val - the numeric value of the changed sample
# prev - the previous sample value
# 
# Make sure the corresponding toggle is enabled in the CHOP Execute DAT.

steps = 64

# RGB sliders
sliders = [
    ['slider2', 'slider3', 'slider4'],    
    ['slider8', 'slider9', 'slider10'],
]

def onValueChange(channel, sampleIndex, val, prev):
    button_val = op('side_bar_out')['RBW'].eval()

    if button_val == 1:
        c = channel.owner
        r = round(c['r'][sampleIndex] * (steps - 1)) / (steps - 1)
        g = round(c['g'][sampleIndex] * (steps - 1)) / (steps - 1)
        b = round(c['b'][sampleIndex] * (steps - 1)) / (steps - 1)

        for group in sliders:
            for i in range(3):
                op(f'GUI/{group[i]}/field1/string')[0, 0] = int([r, g, b][i])
                op('art_net_const').par['value{}'.format(i+1)] = int([r, g, b][i])

    return
