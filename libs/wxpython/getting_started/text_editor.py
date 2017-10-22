# https://wiki.wxpython.org/Getting%20Started
#!/user/bin/env python
import wx
import os


class MyFrame(wx.Frame):
    def __init__(self, *args, **kwargs):
        super(MyFrame, self).__init__(*args, **kwargs)

        self.control = wx.TextCtrl(self, style=wx.TE_MULTILINE)

        self.CreateStatusBar()

        fileMenu = wx.Menu()
        openItem = fileMenu.Append(wx.ID_OPEN, "&Open", "Open a text file")
        aboutItem = fileMenu.Append(wx.ID_ABOUT, "&About", "Information about this program")
        fileMenu.AppendSeparator()
        exitItem = fileMenu.Append(wx.ID_EXIT, "&Exit", "Terminate the program")

        menuBar = wx.MenuBar()
        menuBar.Append(fileMenu, "&File")
        self.SetMenuBar(menuBar)

        # Set events
        self.Bind(wx.EVT_MENU, self.OnOpen, openItem)
        self.Bind(wx.EVT_MENU, self.OnAbout, aboutItem)
        self.Bind(wx.EVT_MENU, self.OnExit, exitItem)

        self.Show(True)

    def OnAbout(self, event):
        wx.MessageBox("A small text editor", "About Sample Editor")

    def OnExit(self, event):
        self.Close(True)

    def OnOpen(self, event):
        self.dirName = ""
        dlg = wx.FileDialog(self, "Choose a file", self.dirName, "", "*.*", wx.FD_OPEN)
        if dlg.ShowModal() == wx.ID_OK:
            self.fileName = dlg.GetFilename()
            self.dirName = dlg.GetDirectory()
            f = open(os.path.join(self.dirName, self.fileName), 'r')
            self.control.SetValue(f.read())
            f.close()
        dlg.Destroy()


app = wx.App(False)
frame = MyFrame(None, title="Small editor")
app.MainLoop()