# Host only libraries ###

if(NOMADZ_TARGET STREQUAL "host" OR NOMADZ_TARGET STREQUAL "host-only")
  # qtpropertybrowser ###

  add_library(qtpropertybrowser STATIC "")

  target_sources(
    qtpropertybrowser
    PRIVATE "qtpropertybrowser/qtbuttonpropertybrowser.cpp"
            "qtpropertybrowser/qteditorfactory.cpp"
            "qtpropertybrowser/qtgroupboxpropertybrowser.cpp"
            "qtpropertybrowser/qtpropertybrowser.cpp"
            "qtpropertybrowser/qtpropertybrowserutils.cpp"
            "qtpropertybrowser/qtpropertybrowserutils_p.h"
            "qtpropertybrowser/qtpropertymanager.cpp"
            "qtpropertybrowser/qttreepropertybrowser.cpp"
            "qtpropertybrowser/qtvariantproperty.cpp"
            "qtpropertybrowser/qtpropertybrowser.qrc")

  target_include_directories(
    qtpropertybrowser PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/qtpropertybrowser/")

  set_target_properties(
    qtpropertybrowser
    PROPERTIES AUTOMOC ON
               AUTORCC ON
               POSITION_INDEPENDENT_CODE ON)

  target_compile_options(qtpropertybrowser PRIVATE -Wno-deprecated-declarations)
  target_link_libraries(qtpropertybrowser PRIVATE ${Qt_LIBRARIES})

  # qxt ###

  add_library(qxt STATIC "")

  target_sources(qxt PRIVATE "libqxt/qxtspanslider.cpp")

  target_include_directories(qxt PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/libqxt/")

  set_target_properties(
    qxt
    PROPERTIES AUTOMOC ON
               AUTORCC ON
               POSITION_INDEPENDENT_CODE ON)

  target_link_libraries(qxt PRIVATE ${Qt_LIBRARIES})
endif()

# snappy ###

add_library(snappy STATIC IMPORTED GLOBAL)
set_target_properties(
  snappy PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                    "${CMAKE_CURRENT_SOURCE_DIR}/snappy/include")
if(NOMADZ_TARGET STREQUAL "v6-32")
  set_target_properties(
    snappy
    PROPERTIES IMPORTED_LOCATION
               "${CMAKE_CURRENT_SOURCE_DIR}/snappy/lib/linux_x86/libsnappy.a")
else() # host, host-only, v6-64
  set_target_properties(
    snappy
    PROPERTIES IMPORTED_LOCATION
               "${CMAKE_CURRENT_SOURCE_DIR}/snappy/lib/linux_x86_64/libsnappy.a"
  )
endif()

# FFTW3 ###

add_library(fftw3 STATIC IMPORTED GLOBAL)
set_target_properties(fftw3 PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                       "${CMAKE_CURRENT_SOURCE_DIR}/fftw-3.3")
if(NOMADZ_TARGET STREQUAL "v6-32")
  set_target_properties(
    fftw3 PROPERTIES IMPORTED_LOCATION
                     "${CMAKE_CURRENT_SOURCE_DIR}/fftw-3.3/Linux32/libfftw3.a")
else() # host, host-only, v6-64
  set_target_properties(
    fftw3 PROPERTIES IMPORTED_LOCATION
                     "${CMAKE_CURRENT_SOURCE_DIR}/fftw-3.3/Linux64/libfftw3.a")
endif()

add_library(mfcc STATIC "libmfcc/src/libmfcc.cpp")
target_include_directories(mfcc PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/fftw-3.3"
  "${CMAKE_CURRENT_SOURCE_DIR}/libmfcc/src")
target_link_libraries(mfcc PUBLIC fftw3)

# Tensorflow lite

if(ENABLE_TFLITE)
  add_subdirectory(tensorflow/tensorflow/lite)

  # Disable warnings
  if(NOMADZ_TARGET STREQUAL "host" OR NOMADZ_TARGET STREQUAL "host-only")
    target_compile_options(
      tensorflow-lite PRIVATE -Wno-deprecated-declarations
                              -Wno-ignored-attributes -Wno-comment)
  else()
    target_compile_options(tensorflow-lite PRIVATE -Wno-deprecated-declarations
                                                   -Wno-attributes)
  endif()
endif()
