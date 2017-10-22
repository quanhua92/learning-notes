import wx

class Singleton(type):
    def __init__(cls, name, bases, dict):
        super(Singleton, cls).__init__(name, bases, dict)
        cls.instance = None

    def __call__(cls, *args, **kwargs):
        if not cls.instance:
            obj = super(Singleton, cls).__call__(*args, **kwargs)
            cls.instance = obj
            cls.instance.SetupWindow()
        return cls.instance


class SingletonDialog(wx.Dialog):
    __metaclass__ = Singleton

    def SetupWindow(self):
        """Hook method for initializing window"""
        self.field = wx.TextCtrl(self)
        self.check = wx.CheckBox(self, label="Enable Foo")

        # Layout
        vsizer = wx.BoxSizer(wx.VERTICAL)
        label = wx.StaticText(self, label="Foobar")
        hsizer = wx.BoxSizer(wx.HORIZONTAL)
        hsizer.AddMany([(label, 0, wx.ALIGN_CENTER_VERTICAL),
                        ((5, 5), 0),
                        (self.field, 0, wx.EXPAND)])
        btnsz = self.CreateButtonSizer(wx.OK)
        vsizer.AddMany([(hsizer, 0, wx.ALL|wx.EXPAND, 10),
                        (self.check, 0, wx.ALL, 10),
                        (btnsz, 0, wx.EXPAND|wx.ALL, 10)])
        self.SetSizer(vsizer)
        self.SetInitialSize()

