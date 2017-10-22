import wx


class HelloFrame(wx.Frame):
    """
    A frame that says Hello World
    """

    def __init__(self, *args, **kwargs):
        super(HelloFrame, self).__init__(*args, **kwargs)

        # a panel in the frame
        panel = wx.Panel(self)

        staticText = wx.StaticText(panel, label="Hello World", pos=(25, 25))
        font = staticText.GetFont()
        font.PointSize += 10
        font = font.Bold()
        staticText.SetFont(font)

        self.makeMenuBar()

        self.CreateStatusBar()
        self.SetStatusText("Welcome to WxPython!")

    def makeMenuBar(self):
        fileMenu = wx.Menu()

        helloItem = fileMenu.Append(-1, "&Hello...\tCtrl-H", "Help string show in status bar for this menu item")
        fileMenu.AppendSeparator()

        exitItem = fileMenu.Append(wx.ID_EXIT)

        helpMenu = wx.Menu()
        aboutItem = helpMenu.Append(wx.ID_ABOUT)

        menuBar = wx.MenuBar()
        menuBar.Append(fileMenu, "&File")
        menuBar.Append(helpMenu, "&Help")

        self.SetMenuBar(menuBar)

        self.Bind(wx.EVT_MENU, self.OnHello, helloItem)
        self.Bind(wx.EVT_MENU, self.OnExit, exitItem)
        self.Bind(wx.EVT_MENU, self.OnAbout, aboutItem)

    def OnExit(self, event):
        self.Close(True)

    def OnHello(self, event):
        wx.MessageBox("Hello again from wxPython")

    def OnAbout(self, event):
        wx.MessageBox("This is a wxPython Hello example", "About Hello World 2", wx.OK | wx.ICON_INFORMATION)


if __name__ == "__main__":
    app = wx.App()
    frame = HelloFrame(None, title="Hello World 2")
    frame.Show()
    app.MainLoop()

