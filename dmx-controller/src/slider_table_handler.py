# me - this DAT.
# 
# dat - the changed DAT
# rows - a list of row indices
# cols - a list of column indices
# cells - the list of cells that have changed content
# prev - the list of previous string contents of the changed cells
# 
# Make sure the corresponding toggle is enabled in the DAT Execute DAT.
# 
# If rows or columns are deleted, sizeChange will be called instead of row/col/cellChange.


def onTableChange(dat):

    # DMX信号を送信
    dmx_controller = mod('/project1/dmx_controller')
    index = op('/project1/GUI/label_table')[int(parent(2).name.split('r')[-1]) - 1, 1]
    dmx_controller.cv[index] = int(dat[0, 0].val)
    
    if op('/project1/side_bar/null1')['v3'] == 1:
        dmx_controller.send()
    
    # デバッグ出力
    print(f'{index}: {dmx_controller.cv[index]}')

    return
