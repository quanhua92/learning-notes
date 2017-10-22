import wx


class ExamplePanel(wx.Panel):
    def __init__(self, *args, **kw):
        super(ExamplePanel, self).__init__(*args, **kw)

        self.quote = wx.StaticText(self,
                                   label="Your quote: ",
                                   pos=(20, 30))
        # we may want to use a wx.Sizer instead of hard-code position

        # a multiline TextCtrl to show how events work
        self.logger = wx.TextCtrl(self, pos=(400, 20), size=(200, 300),
                                  style=wx.TE_MULTILINE | wx.TE_READONLY)

        # A button
        self.button = wx.Button(self, label="Save", pos=(200, 325))
        self.Bind(wx.EVT_BUTTON, self.OnClick, self.button)

        # The edit control
        self.lblName = wx.StaticText(self, label="Your name: ", pos=(20, 60))
        self.editName = wx.TextCtrl(self, value="Enter here your name",
                                    pos=(200, 60),
                                    size=(140, -1))

        self.Bind(wx.EVT_TEXT, self.OnChange, self.editName)
        self.Bind(wx.EVT_CHAR, self.OnKeyPress, self.editName)

        # The combox control
        self.sampleList = ['friends', 'advertising', 'web search', 'Yellow Pages']
        self.lblHear = wx.StaticText(self, label="How did you year from us?", pos=(20, 90))
        self.editHear = wx.ComboBox(self, pos=(200, 90), size=(140, -1), choices=self.sampleList, style=wx.CB_DROPDOWN)

        self.Bind(wx.EVT_COMBOBOX, self.EvtComboBox, self.editHear)
        self.Bind(wx.EVT_TEXT, self.EvtText, self.editHear)

        # Checkbox
        self.insure = wx.CheckBox(self, label="Do you want Insured Shipment?", pos=(20, 180))
        self.Bind(wx.EVT_CHECKBOX, self.EvtCheckBox, self.insure)

        # Radio Boxes
        radioList = ['blue', 'red', 'yellow', 'orange', 'green', 'purple', 'navy blue', 'black', 'gray']
        self.rb = wx.RadioBox(self, label="What color would you like?",
                              pos=(20, 210), choices=radioList, majorDimension=3, style=wx.RA_SPECIFY_COLS)
        self.Bind(wx.EVT_RADIOBOX, self.EvtRadioBox, self.rb)

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
    ExamplePanel(frame)
    frame.Show()
    app.MainLoop()