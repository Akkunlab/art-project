# me - this DAT
# panelValue - the PanelValue object that changed
# prev - the previous value of the PanelValue object that changed
# Make sure the corresponding toggle is enabled in the Panel Execute DAT.

def onValueChange(panelValue):

    if panelValue:
        op('/project1/udmx').run()

    return
