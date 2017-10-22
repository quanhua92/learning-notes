import wx

from pubsub import pub

print("pubsub API version", pub.VERSION_API)

from pubsub.utils.notification import useNotifyByWriteFile
import sys

useNotifyByWriteFile(sys.stdout)

from wx_win1 import View
from wx_win2 import ChangerWidget


class Model:
    def __init__(self):
        self.myMoney = 0

    def addMoney(self, value):
        self.myMoney += value

        pub.sendMessage("money_changed", money=self.myMoney)

    def removeMoney(self, value):
        self.myMoney -= value

        pub.sendMessage("money_changed", money=self.myMoney)


class Controller:
    def __init__(self):
        self.model = Model()
        self.view1 = View()
        self.view1.setMoney(self.model.myMoney)

        self.view2 = ChangerWidget()

        self.view1.Show()
        self.view2.Show()

        pub.subscribe(self.changeMoney, "money_changing")

    def changeMoney(self, amount):
        if amount > 0:
            self.model.addMoney(amount)
        else:
            self.model.removeMoney(-amount)


if __name__ == "__main__":
    app = wx.App()
    c = Controller()
    sys.stdout = sys.__stdout__

    app.MainLoop()

