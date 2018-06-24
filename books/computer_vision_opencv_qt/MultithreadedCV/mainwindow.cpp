#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // videoprocessorthread
//    connect(&processor,
//            SIGNAL(inDisplay(QPixmap)),
//            ui->inVideo,
//            SLOT(setPixmap(QPixmap)));

//    connect(&processor,
//            SIGNAL(outDisplay(QPixmap)),
//            ui->outVideo,
//            SLOT(setPixmap(QPixmap)));
//    processor.start();

    // VideoProcessor
    processor = new VideoProcessor();
    processor->moveToThread(new QThread(this));

    connect(processor->thread(),
            &QThread::started,
            processor,
            &VideoProcessor::startVideo);

    connect(processor->thread(),
            &QThread::finished,
            processor,
            &VideoProcessor::deleteLater);

    connect(processor,
            &VideoProcessor::inDisplay,
            ui->inVideo,
            &QLabel::setPixmap);

    connect(processor,
            &VideoProcessor::outDisplay,
            ui->outVideo,
            &QLabel::setPixmap);
    processor->thread()->start();
}

MainWindow::~MainWindow()
{
    // videoprocessorthread
//    processor.requestInterruption();
//    processor.wait();

    // videoprocessor
    processor->stopVideo();
    processor->thread()->quit();
    processor->thread()->wait();
    delete ui;
}
