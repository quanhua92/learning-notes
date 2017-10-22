import wx
from pubsub import pub


class View(wx.Frame):
    def __init__(self, parent=None):
        super(View, self).__init__(parent, -1, "Main View")

        sizer = wx.BoxSizer(wx.VERTICAL)
        text = wx.StaticText(self, -1, "My Money")
        ctrl = wx.TextCtrl(self, -1, "")

        sizer.Add(text, 0, wx.EXPAND | wx.ALL)
        sizer.Add(ctrl, 0, wx.EXPAND | wx.ALL)

        self.moneyCtrl = ctrl
        ctrl.SetEditable(False)
        self.SetSizer(sizer)

        pub.subscribe(self.setMoney, "money_changed")

    def setMoney(self, money):
        self.moneyCtrl.SetValue(str(money))