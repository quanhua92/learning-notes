from __future__ import division
import sys
from math import *
from PyQt4.QtCore import *
from PyQt4.QtGui import *


class Form(QDialog):
    def __init__(self, parent=None):
        super(Form, self).__init__(parent)
        self.browser = QTextBrowser()
        self.lineEdit = QLineEdit("Type an expression and press Enter")
        self.lineEdit.selectAll()
        layout = QVBoxLayout()
        layout.addWidget(self.browser)
        layout.addWidget(self.lineEdit)

        self.setLayout(layout)
        self.lineEdit.setFocus()
        self.connect(self.lineEdit, SIGNAL("returnPressed()"), self.update_ui)
        self.setWindowTitle("Calculate")

    def update_ui(self):
        text = unicode(self.lineEdit.text())
        try:
            self.browser.append("%s = <b>%s</b>" % (text, eval(text)))
        except:
            self.browser.append("<font color=red>%s is invalid!</font>" % text)


app = QApplication(sys.argv)
form = Form()
form.show()
app.exec_()
