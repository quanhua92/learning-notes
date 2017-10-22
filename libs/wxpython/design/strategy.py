import wx


class BaseDialogStrategy:
    def GetWindowObject(self, parent):
        raise NotImplementedError, "required Method"

    def DoOnOk(self):
        """:return bool (True to exit, False to not)"""
        return True

    def DoOnCancel(self):
        """:return bool (True to exit, False to not)"""
        return True


class StrategyDialog(wx.Dialog):
    def __init__(self, parent, strategy, *args, **kwargs):
        super(StrategyDialog, self).__init__(parent, *args, **kwargs)

        # Attributes
        self.strategy = strategy
        self.pane = self.strategy.GetWindowObject(self)

        # Layout
        self.__DoLayout()

        # Event Handlers
        self.Bind(wx.EVT_BUTTON, self.OnButton)

    def __DoLayout(self):
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.pane, 1, wx.EXPAND)
        btnsz = self.CreateButtonSizer(wx.OK | wx.CANCEL)
        sizer.Add(btnsz, 0, wx.EXPAND|wx.ALL, 8)
        self.SetSizer(sizer)

    def GetStrategy(self):
        return self.strategy

    def OnButton(self, event):
        evt_id = event.GetId()
        bCanExit = False
        if evt_id == wx.ID_OK:
            bCanExit = self.strategy.DoOnOk()
        elif evt_id == wx.ID_CANCEL:
            bCanExit = self.strategy.DoOnCancel()
        else:
            event.Skip()
        if bCanExit:
            self.EndModal(evt_id)


class FileTreeStrategy(BaseDialogStrategy):
    """File chooser strategy"""
    def GetWindowObject(self, parent):
        assert not hasattr(self, 'dirctrl')
        self.dirctrl = wx.GenericDirCtrl(parent)
        return self.dirctrl

    def DoOnOk(self):
        path = self.dirctrl.GetPath()
        if path:
            wx.MessageBox("You selected: %s" % path)
            return True
        else:
            wx.MessageBox("No file selected!")
            return False


if __name__ == "__main__":
    app = wx.App()
    frame = wx.Frame()
    strategy = FileTreeStrategy()
    dlg = StrategyDialog(frame, strategy, title="Choose file")
    dlg.ShowModal()
