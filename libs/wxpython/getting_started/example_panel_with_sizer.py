import wx


class ExamplePanelWithSizer(wx.Panel):
    def __init__(self, *args, **kw):
        super(ExamplePanelWithSizer, self).__init__(*args, **kw)

        # create some sizers
        mainSizer = wx.BoxSizer(wx.VERTICAL)
        grid = wx.GridBagSizer(hgap=5, vgap=5)
        hSizer = wx.BoxSizer(wx.HORIZONTAL)

        self.quote = wx.StaticText(self, label="Your quote: ")
        grid.Add(self.quote, pos=(0,0 ))

        # a multiline TextCtrl to show how events work
        self.logger = wx.TextCtrl(self, pos=(400, 20), size=(200, 300),
                                  style=wx.TE_MULTILINE | wx.TE_READONLY)

        # A button
        self.button = wx.Button(self, label="Save", pos=(200, 325))
        self.Bind(wx.EVT_BUTTON, self.OnClick, self.button)

        # The edit control
        self.lblName = wx.StaticText(self, label="Your name: ", pos=(20, 60))
        grid.Add(self.lblName, pos=(1, 0))
        self.editName = wx.TextCtrl(self, value="Enter here your name", size=(140, -1))
        grid.Add(self.editName, pos=(1, 1))

        self.Bind(wx.EVT_TEXT, self.OnChange, self.editName)
        self.Bind(wx.EVT_CHAR, self.OnKeyPress, self.editName)

        # The combox control
        self.sampleList = ['friends', 'advertising', 'web search', 'Yellow Pages']
        self.lblHear = wx.StaticText(self, label="How did you year from us?", pos=(20, 90))
        grid.Add(self.lblHear, pos=(3, 0))
        self.editHear = wx.ComboBox(self, pos=(200, 90), size=(140, -1), choices=self.sampleList, style=wx.CB_DROPDOWN)
        grid.Add(self.editHear, pos=(3, 1))

        self.Bind(wx.EVT_COMBOBOX, self.EvtComboBox, self.editHear)
        self.Bind(wx.EVT_TEXT, self.EvtText, self.editHear)

        grid.Add((10, 40), pos=(2, 0))

        # Checkbox
        self.insure = wx.CheckBox(self, label="Do you want Insured Shipment?", pos=(20, 180))
        grid.Add(self.insure, pos=(4, 0), span=(1, 2), flag=wx.BOTTOM, border=5)
        self.Bind(wx.EVT_CHECKBOX, self.EvtCheckBox, self.insure)

        # Radio Boxes
        radioList = ['blue', 'red', 'yellow', 'orange', 'green', 'purple', 'navy blue', 'black', 'gray']
        self.rb = wx.RadioBox(self, label="What color would you like?",
                              pos=(20, 210), choices=radioList, majorDimension=3, style=wx.RA_SPECIFY_COLS)
        grid.Add(self.rb, pos=(5, 0), span=(1, 2))
        self.Bind(wx.EVT_RADIOBOX, self.EvtRadioBox, self.rb)

        hSizer.Add(grid, 0, wx.ALL, 5)
        hSizer.Add(self.logger)
        mainSizer.Add(hSizer, 0, wx.ALL, 5)
        mainSizer.Add(self.button, 0, wx.CENTER)
        self.SetSizerAndFit(mainSizer)

    def EvtRadioBox(self, e):
        self.logger.AppendText('EvtRadioBox: %d\n' % e.GetInt())

    def EvtComboBox(self, e):
        self.logger.AppendText('EvtComboBox: %s\n' % e.GetString())

    def OnClick(self, e):
        self.logger.AppendText('Click on object with Id %d\n' % e.GetId())

    def OnChange(self, e):
        self.logger.AppendText("EditName: OnChange %s\n" % e.GetString())

    def OnKeyPress(self, e):
        self.logger.AppendText("EditName: OnKeyPress %s\n" % e.GetKeyCode())
        e.Skip()

    def EvtText(self, e):
        self.logger.AppendText("EvtText: %s\n" % e.GetString())

    def EvtChar(self, e):
        self.logger.AppendText("EvtChar: %d\n" % e.GetKeyCode())
        e.Skip()

    def EvtCheckBox(self, e):
        self.logger.AppendText("EvtCheckBox: %d\n" % e.IsChecked())


if __name__ == "__main__":
    app = wx.App(False)
    frame = wx.Frame(None)
    ExamplePanelWithSizer(frame)
    frame.Show()
    app.MainLoop()