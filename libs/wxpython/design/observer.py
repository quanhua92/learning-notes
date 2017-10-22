import wx

from pubsub import pub
from pubsub.utils.notification import useNotifyByWriteFile
import sys
useNotifyByWriteFile(sys.stdout)

MSG_CONFIG_ROOT = ("config_", )
MSG_CONFIG_FONT = "font"


class Configuration(object):
    def __init__(self):
        super(Configuration, self).__init__()
        self.data = dict()

    def SetValue(self, key, value):
        self.data[key] = value
        pub.sendMessage(MSG_CONFIG_ROOT + (key,), data=value)

    def GetValue(self, key):
        return self.data.get(key, None)


class ObserverApp(wx.App):
    def OnInit(self):
        self.config = Configuration()
        self.frame = ObserverFrame(None, title="Observer Pattern")
        self.frame.Show()

        self.configdlg = ConfigDialog(self.frame, title="Config Dialog")
        self.configdlg.Show()
        return True

    def GetConfig(self):
        return self.config


class ConfigDialog(wx.Dialog):
    def __init__(self, *args, **kwargs):
        super(ConfigDialog, self).__init__(*args, **kwargs)

        self.panel = ConfigPanel(self)

        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.panel, 1, wx.EXPAND)
        self.SetSizer(sizer)
        self.SetInitialSize((300, 300))


class ConfigPanel(wx.Panel):
    def __init__(self, parent):
        super(ConfigPanel, self).__init__(parent)

        self.picker = wx.FontPickerCtrl(self)
        self.__DoLayout()

        self.Bind(wx.EVT_FONTPICKER_CHANGED, self.OnFontPicker)

    def __DoLayout(self):
        vsizer = wx.BoxSizer(wx.VERTICAL)
        hsizer = wx.BoxSizer(wx.HORIZONTAL)

        vsizer.AddStretchSpacer()
        hsizer.AddStretchSpacer()
        hsizer.Add(self.picker)
        hsizer.AddStretchSpacer()
        vsizer.Add(hsizer, 0, wx.EXPAND)
        vsizer.AddStretchSpacer()
        self.SetSizer(vsizer)

    def OnFontPicker(self, e):
        font = self.picker.GetSelectedFont()
        config = wx.GetApp().GetConfig()
        config.SetValue(MSG_CONFIG_FONT, font)


class ObserverFrame(wx.Frame):
    def __init__(self, *args, **kwargs):
        super(ObserverFrame, self).__init__(*args, **kwargs)
        self.txt = wx.TextCtrl(self, style=wx.TE_MULTILINE)
        self.txt.SetValue("Change the font in config dialog and see it update here")

        pub.subscribe(self.OnConfigMsg, MSG_CONFIG_ROOT)

    def OnConfigMsg(self, data, obj=pub.AUTO_TOPIC):
        topics = obj.getName().split(".")
        if topics[-1] == "font":
            self.SetFont(data)
            self.txt.SetFont(data)


if __name__ == "__main__":
    app = ObserverApp()
    app.MainLoop()

