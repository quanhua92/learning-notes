import os
import time
import wx


class LoggerMixin:
    def __init__(self, logfile="log.txt"):
        self.logfile = logfile

    def Log(self, msg):
        if self.logfile is None:
            return

        with open(self.logfile, 'ab') as handle:
            ltime = time.localtime(time.time())
            tstamp = "%s:%s:%s" % (
                str(ltime[3]).zfill(2),
                str(ltime[4]).zfill(2),
                str(ltime[5]).zfill(2)
            )

            client = getattr(self, 'GetName', lambda: "unknown")()
            output = "[%s][%s] %s%s" % (tstamp, client, msg, os.linesep)
            handle.write(output)


class MixinRecipeFrame(wx.Frame, LoggerMixin):
    def __init__(self, *args, **kwargs):
        super(MixinRecipeFrame, self).__init__(*args, **kwargs)
        LoggerMixin.__init__(self)
        self.Log("Creating instance...")

        # Attributes
        self.panel = MixinRecipePanel(self)

        # Layout
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.panel, 1, wx.EXPAND)
        self.SetSizer(sizer)
        self.SetInitialSize((300, 300))


class MixinRecipePanel(wx.Panel, LoggerMixin):
    def __init__(self, *args, **kwargs):
        super(MixinRecipePanel, self).__init__(*args, **kwargs)
        LoggerMixin.__init__(self)

        self.Log("Creating instance")


if __name__ == "__main__":
    app = wx.App()
    frame = MixinRecipeFrame(None, title="Mixin")
    frame.Show()
    app.MainLoop()
