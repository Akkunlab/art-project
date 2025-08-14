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
    button_val = op('/project1/side_bar_out')['Out'].eval()
    val = int(dat[0, 0].val)

    # DMX信号を送信
    udmx = mod('/project1/udmx')
    art_net = op('/project1/art_net_const')
    index = op('/project1/GUI/label_table')[int(parent(2).name.split('r')[-1]) - 1, 1]
    udmx.cv[index] = int(val)

    if button_val:
        getattr(art_net.par, f'value{index}').val = int(val)
        udmx.send()
    
    # デバッグ出力
    print(f'{index}: {val}')

    return
