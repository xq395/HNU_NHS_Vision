add_library(detection
    detector.cpp
    preprocessor.cpp
    traditional_detector.cpp
)

if(onnxruntime_FOUND)
    target_sources(detection PRIVATE yolo_detector.cpp)
    target_compile_definitions(detection PUBLIC HAS_ONNX_RUNTIME)
    target_link_libraries(detection PUBLIC onnxruntime)
endif()

target_include_directories(detection PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(detection PUBLIC
    common
    ${OpenCV_LIBS}
)
