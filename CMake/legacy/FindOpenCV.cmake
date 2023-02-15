set(OpenCV_LIBS
  "${CMAKE_SOURCE_DIR}/Util/opencv3/nao32/lib/libopencv_xfeatures2d.a"
  "${CMAKE_SOURCE_DIR}/Util/opencv3/nao32/lib/libopencv_ml.a"
  "${CMAKE_SOURCE_DIR}/Util/opencv3/nao32/lib/libopencv_objdetect.a"
  "${CMAKE_SOURCE_DIR}/Util/opencv3/nao32/lib/libopencv_calib3d.a"
  "${CMAKE_SOURCE_DIR}/Util/opencv3/nao32/lib/libopencv_features2d.a"
  "${CMAKE_SOURCE_DIR}/Util/opencv3/nao32/lib/libopencv_highgui.a"
  "${CMAKE_SOURCE_DIR}/Util/opencv3/nao32/lib/libopencv_imgcodecs.a"
  "${CMAKE_SOURCE_DIR}/Util/opencv3/nao32/lib/libopencv_flann.a"
  "${CMAKE_SOURCE_DIR}/Util/opencv3/nao32/lib/libopencv_imgproc.a"
  "${CMAKE_SOURCE_DIR}/Util/opencv3/nao32/lib/libopencv_core.a"
  "${CMAKE_SOURCE_DIR}/Util/opencv3/nao32/share/OpenCV/3rdparty/lib/liblibjasper.a"
  "${CMAKE_SOURCE_DIR}/Util/opencv3/nao32/share/OpenCV/3rdparty/lib/liblibpng.a"
  "${CMAKE_SOURCE_DIR}/Util/opencv3/nao32/share/OpenCV/3rdparty/lib/liblibtiff.a"
  "${CMAKE_SOURCE_DIR}/Util/opencv3/nao32/share/OpenCV/3rdparty/lib/libzlib.a"
  "${CMAKE_SOURCE_DIR}/Util/opencv3/nao32/share/OpenCV/3rdparty/lib/liblibwebp.a"
  CACHE INTERNAL "" FORCE)
include_directories("${CMAKE_SOURCE_DIR}/Util/opencv3/nao32/include/" SYSTEM) # FIXME
