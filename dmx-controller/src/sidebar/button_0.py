# me - this DAT
# panelValue - the PanelValue object that changed
# prev - the previous value of the PanelValue object that changed
# Make sure the corresponding toggle is enabled in the Panel Execute DAT.

def onOffToOn(panelValue):
    include_sliders = [
        'slider1', 'slider2', 'slider3', 'slider4',
        'slider7', 'slider8', 'slider9', 'slider10',
        'slider20',
        'slider32',
        'slider37', 'slider38', 'slider39', 'slider40',
    ]
    gui_base_path = '/project1/GUI'
    gui = op(gui_base_path)

    # GUI 内のすべてのスライダーを検索
    for slider in gui.findChildren(name='slider*', type=COMP):

        # スキップ
        if slider.name not in include_sliders:
            continue  

        field = slider.op('field1/string')

        if field is not None:
            field[0, 0] = '0'

    return
