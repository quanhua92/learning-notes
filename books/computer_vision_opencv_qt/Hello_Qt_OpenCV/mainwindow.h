#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QDir>
#include <QFile>
#include <QMessageBox>
#include <QSettings>
#include <QCloseEvent>

#include <opencv2/opencv.hpp>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_inputPushButton_pressed();

    void on_outputPushButton_pressed();

protected:
    void closeEvent(QCloseEvent *event);

private:
    void loadSettings();
    void saveSettings();
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
