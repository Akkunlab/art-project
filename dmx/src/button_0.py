# me - this DAT
# panelValue - the PanelValue object that changed
# prev - the previous value of the PanelValue object that changed
# Make sure the corresponding toggle is enabled in the Panel Execute DAT.

def onOffToOn(panelValue):
    skip_sliders = ['slider13', 'slider14', 'slider19', 'slider20']
    gui_base_path = '/project1/GUI'
    gui = op(gui_base_path)

    # GUI 内のすべてのスライダーを検索
    for slider in gui.findChildren(name='slider*', type=COMP):

        # スキップ
        if slider.name in skip_sliders:
            continue  

        field = slider.op('field1/string')

        if field is not None:
            field[0, 0] = '0'

    return
