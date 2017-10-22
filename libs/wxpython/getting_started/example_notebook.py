import wx
from example_panel import ExamplePanel

app = wx.App(False)
frame = wx.Frame(None, title="Demo with Notebook")
nb = wx.Notebook(frame)

nb.AddPage(ExamplePanel(nb), "Absolute Position")
nb.AddPage(ExamplePanel(nb), "Page Two")
nb.AddPage(ExamplePanel(nb), "Page Three")
frame.Show()

app.MainLoop()
