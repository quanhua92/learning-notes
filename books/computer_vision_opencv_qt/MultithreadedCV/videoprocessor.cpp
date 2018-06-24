#include "videoprocessor.h"
#include <opencv2/opencv.hpp>

VideoProcessor::VideoProcessor(QObject *parent) : QObject(parent)
{

}

void VideoProcessor::startVideo()
{
    using namespace cv;

    VideoCapture camera(1);
    Mat inFrame, outFrame;
    stopped = false;

    while(camera.isOpened() && !stopped){
        camera >> inFrame;

        if(inFrame.empty())
            continue;
        bitwise_not(inFrame, outFrame);

        emit inDisplay(
             QPixmap::fromImage(
                        QImage(
                            inFrame.data,
                            inFrame.cols,
                            inFrame.rows,
                            inFrame.step,
                            QImage::Format_RGB888
                        ).rgbSwapped()));
        emit outDisplay(
                    QPixmap::fromImage(
                        QImage(
                            outFrame.data,
                            outFrame.cols,
                            outFrame.rows,
                            outFrame.step,
                            QImage::Format_RGB888
                        ).rgbSwapped()));
    }
}

void VideoProcessor::stopVideo()
{
    // need mutex
    stopped = true;
}
