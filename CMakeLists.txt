add_library(solve
    solver.cpp
    tracker.cpp
    predictor.cpp
    planner.cpp
    extrinsics.cpp
)

target_include_directories(solve PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(solve PUBLIC
    common
    io
    detection
    ${OpenCV_LIBS}
    Eigen3::Eigen
)
