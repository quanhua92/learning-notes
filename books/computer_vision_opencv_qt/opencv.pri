INCLUDEPATH += D:/workspace/vcpkg/installed/x64-windows/include 
Debug: { 
LIBS += -LD:/workspace/vcpkg/installed/x64-windows/debug/lib
LIBS += -lopencv_calib3d341d
LIBS += -lopencv_core341d
LIBS += -lopencv_features2d341d
LIBS += -lopencv_flann341d
LIBS += -lopencv_highgui341d
LIBS += -lopencv_imgcodecs341d
LIBS += -lopencv_imgproc341d
LIBS += -lopencv_ml341d
LIBS += -lopencv_objdetect341d
LIBS += -lopencv_photo341d
LIBS += -lopencv_shape341d
LIBS += -lopencv_stitching341d
LIBS += -lopencv_superres341d
LIBS += -lopencv_video341d
LIBS += -lopencv_videoio341d
LIBS += -lopencv_videostab341d
} 
Release: { 
LIBS += -LD:/workspace/vcpkg/installed/x64-windows/lib
LIBS += -lopencv_calib3d341
LIBS += -lopencv_core341
LIBS += -lopencv_features2d341
LIBS += -lopencv_flann341
LIBS += -lopencv_highgui341
LIBS += -lopencv_imgcodecs341
LIBS += -lopencv_imgproc341
LIBS += -lopencv_ml341
LIBS += -lopencv_objdetect341
LIBS += -lopencv_photo341
LIBS += -lopencv_shape341
LIBS += -lopencv_stitching341
LIBS += -lopencv_superres341
LIBS += -lopencv_video341
LIBS += -lopencv_videoio341
LIBS += -lopencv_videostab341
}



