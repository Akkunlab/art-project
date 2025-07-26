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
# If rows or columns are deleted, sizeChange will be called instead of rowcolcellChange.

def onTableChange(dat):
    merge = op('merge1')
    rename = op('rename1')
    label_table = op('label_table')

    from_names = [ch.name for ch in merge.chans()]
    to_names = [label_table[i, 0].val for i in range(len(from_names))]

    if len(from_names) != len(to_names):
        raise Warning(f'The number of channels ({len(from_names)}) does not match the number of labels ({len(to_names)}).')

    rename.par.renamefrom = ' '.join(from_names)
    rename.par.renameto = ' '.join(to_names)

    return
