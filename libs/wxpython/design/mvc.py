import wx


class ModelInterface(object):
    """"Defines an interface for a simple value generator model"""
    def __init__(self):
        super(ModelInterface, self).__init__()

        self.value = 0
        self.observers = list()

    def Generate(self):
        """Interface method to be implemented by subclasses"""
        raise NotImplementedError

    def SetValue(self, value):
        self.value = value
        self.NotifyObservers()

    def GetValue(self):
        return self.value

    def RegisterObserver(self, callback):
        """Register an observer callback
        :param callable(newvalue)
        """
        self.observers.append(callback)

    def NotifyObservers(self):
        """Notify all observers of current value"""
        for observer in self.observers:
            observer()


class ControllerInterface(object):
    """Defines an interface a value generator controller"""
    def __init__(self, model):
        super(ControllerInterface, self).__init__()

        # Attributes
        self.model = model
        self.view = TheView(None, self, self.model, "Fibonacci Generator")

        # Setup
        self.view.Show()

    def DoGenerateNext(self):
        """User action request next value"""
        raise NotImplementedError


class FibonacciModel(ModelInterface):
    def Generate(self):
        cval = self.GetValue()
        # Get the next one
        for fib in self.fibonacci():
            if fib > cval:
                self.SetValue(fib)
                break

    @staticmethod
    def fibonacci():
        """FIbonacci generator method"""
        a, b = 0, 1
        while True:
            yield a
            a, b = b, a + b


class FibonacciController(ControllerInterface):
    def DoGenerateNext(self):
        self.view.EnableButton(False)
        self.model.Generate()


class TheView(wx.Frame):
    def __init__(self, parent, controller, model, title, *args, **kwargs):
        super(TheView, self).__init__(parent, title=title, *args, **kwargs)

        # Attributes
        self.panel = ViewPanel(self, controller, model)

        # Layout
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.panel, 1, wx.EXPAND)
        self.SetSizer(sizer)
        self.SetInitialSize((300, 300))

    def EnableButton(self, enable=True):
        self.panel.button.Enable(enable)


class ViewPanel(wx.Panel):
    def __init__(self, parent, controller, model):
        super(ViewPanel, self).__init__(parent)

        # Attributes
        self.model = model
        self.controller = controller
        initial = str(self.model.GetValue())
        self.text = wx.TextCtrl(self, value=initial)
        self.button = wx.Button(self, label="Generate")

        # Layout
        self.__DoLayout()

        # Setup
        self.model.RegisterObserver(self.OnModelUpdate)

        # Event Handlers
        self.Bind(wx.EVT_BUTTON, self.OnAction)

    def __DoLayout(self):
        vsizer = wx.BoxSizer(wx.VERTICAL)
        hsizer = wx.BoxSizer(wx.HORIZONTAL)

        vsizer.AddStretchSpacer()
        vsizer.Add(self.text, 0, wx.ALIGN_CENTER|wx.ALL, 8)
        hsizer.AddStretchSpacer()
        hsizer.Add(self.button)
        hsizer.AddStretchSpacer()
        vsizer.Add(hsizer, 0, wx.EXPAND)
        vsizer.AddStretchSpacer()
        self.SetSizer(vsizer)

    def OnModelUpdate(self):
        """Observer Method"""
        value = self.model.GetValue()
        self.text.SetValue(str(value))
        self.button.Enable(True)

    def OnAction(self, event):
        self.controller.DoGenerateNext()


class ModelViewApp(wx.App):
    def OnInit(self):
        self.model = FibonacciModel()
        self.controller = FibonacciController(self.model)
        return True


if __name__ == "__main__":
    app = ModelViewApp(False)
    app.MainLoop()
