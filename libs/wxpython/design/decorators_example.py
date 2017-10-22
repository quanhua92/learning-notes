import wx


class expose(object):
    """Expose a panels method to a specified class.
    The panel that is having its method exposed by this decorator must be
    a child of the class its exposing itself too."""
    def __init__(self, cls):
        """@param cls: class to expose the method to"""
        super(expose, self).__init__()
        self.cls = cls

    def __call__(self, funct):
        """Dynamically bind and expose the function to the top level window class"""
        fname = funct.func_name

        def delegate(*args, **kwargs):
            """Delegate method for panel"""
            self = args[0]  # The TLW
            # Find the panel this method belongs to
            panel = None
            for child in self.GetChildren():
                if isinstance(child, wx.Panel) and hasattr(child, fname):
                    panel = child
                    break
            assert panel is not None, "No matching child!"
            # Call the panels method
            return getattr(panel, fname)(*args[1:], **kwargs)
        # Bind the new delegate method to the ltw class
        delegate.__name__ = funct.__name__
        delegate.__doc__ = funct.__doc__
        setattr(self.cls, fname, delegate)

        # Return original function to the current class
        return funct


class CommentDialog(wx.Dialog):
    def __init__(self, *args, **kwargs):
        super(CommentDialog, self).__init__(*args, **kwargs)

        # Attribute
        self.panel = CommentPanel(self)

        # Layout
        self.__DoLayout()

    def __DoLayout(self):
        vsizer = wx.BoxSizer(wx.VERTICAL)
        vsizer.Add(self.panel, 1, wx.EXPAND)
        btnsz = self.CreateButtonSizer(wx.OK|wx.CANCEL)
        vsizer.Add(btnsz, 0, wx.EXPAND|wx.ALL, 8)
        self.SetSizer(vsizer)


class CommentPanel(wx.Panel):
    def __init__(self, *args, **kwargs):
        super(CommentPanel, self).__init__(*args, **kwargs)

        # Attributes
        self.title = wx.TextCtrl(self)
        self.comment = wx.TextCtrl(self, style=wx.TE_MULTILINE)

        # Layout
        self.__DoLayout()

    def __DoLayout(self):
        vsizer = wx.BoxSizer(wx.VERTICAL)

        tsizer = wx.BoxSizer(wx.HORIZONTAL)
        tsizer.Add(wx.StaticText(self, label="Title:"), 0, wx.RIGHT, 5)
        tsizer.Add(self.title, 1, wx.EXPAND)

        vsizer.Add(tsizer, 0, wx.EXPAND | wx.ALL, 5)
        vsizer.Add(self.comment, 1, wx.EXPAND|wx.ALL, 5)
        self.SetSizer(vsizer)

    @expose(CommentDialog)
    def GetCommentTitle(self):
        return self.title.GetValue()

    @expose(CommentDialog)
    def SetCommentTitle(self, title):
        self.title.SetValue(title)

    @expose(CommentDialog)
    def GetComment(self):
        return self.comment.GetValue()

    @expose(CommentDialog)
    def SetComment(self, comment):
        self.comment.SetValue(comment)


class DynamicMethodFrame(wx.Frame):
    def __init__(self, *args, **kwargs):
        super(DynamicMethodFrame, self).__init__(*args, **kwargs)

        self.panel = DynamicMethodPanel(self)

        # Layout
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.panel, 1, wx.EXPAND)
        self.SetSizer(sizer)
        self.SetInitialSize((300, 300))


class DynamicMethodPanel(wx.Panel):
    def __init__(self, *args, **kwargs):
        super(DynamicMethodPanel, self).__init__(*args, **kwargs)

        # Attributes
        self.button = wx.Button(self, label="Get Comment")
        # Setup
        self.__DoLayout()
        # Event handler
        self.Bind(wx.EVT_BUTTON, self.OnGetComment, self.button)

    def __DoLayout(self):
        vsizer = wx.BoxSizer(wx.VERTICAL)
        hsizer = wx.BoxSizer(wx.HORIZONTAL)
        hsizer.AddStretchSpacer()
        hsizer.Add(self.button)
        hsizer.AddStretchSpacer()
        vsizer.AddStretchSpacer()
        vsizer.Add(hsizer, 0, wx.EXPAND)
        vsizer.AddStretchSpacer()
        self.SetSizer(vsizer)

    def OnGetComment(self, event):
        dlg = CommentDialog(self)
        dlg.SetCommentTitle("Comment Title")
        dlg.SetComment("Enter Comment Here")

        if dlg.ShowModal() == wx.ID_OK:
            print("OK Clicked")
            print("Title: ", dlg.GetCommentTitle())
            print("Comment: ", dlg.GetComment())
        else:
            print("Cancel clicked")


if __name__ == "__main__":
    app = wx.App()
    frame = DynamicMethodFrame(None, title="Dynamic Method frame")
    frame.Show()
    app.MainLoop()



