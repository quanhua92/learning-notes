import wx
from pubsub import pub


class ChangerWidget(wx.Frame):
    CHANGE = 10

    def __init__(self, parent=None):
        super(ChangerWidget, self).__init__(parent, -1, "Changer View")

        sizer = wx.BoxSizer(wx.VERTICAL)
        self.add = wx.Button(self, -1, "Add Money")
        self.remove = wx.Button(self, -1, "Remove Money")

        sizer.Add(self.add, 0, wx.EXPAND | wx.ALL)
        sizer.Add(self.remove, 0, wx.EXPAND | wx.ALL)
        self.SetSizer(sizer)

        self.add.Bind(wx.EVT_BUTTON, self.onAdd)
        self.remove.Bind(wx.EVT_BUTTON, self.onRemove)

    def onAdd(self, e):
        print("----------")
        pub.sendMessage("money_changing", amount=self.CHANGE)

    def onRemove(self, e):
        pub.sendMessage("money_changing", amount=-self.CHANGE)
