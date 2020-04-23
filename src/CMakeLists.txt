# Copyright 2020, CNRS
# See LICENSE file.

ADD_EXECUTABLE(rohans_test-cg rohans_test-cg.cpp)
TARGET_LINK_LIBRARIES(rohans_test-cg pinocchio::pinocchio ${CMAKE_DL_LIBS})

ADD_EXECUTABLE(segment_segment_distance segment_segment_distance.cpp)
TARGET_INCLUDE_DIRECTORIES(segment_segment_distance PRIVATE SYSTEM ${EIGEN3_INCLUDE_DIR})

ADD_EXECUTABLE(polynom_codegen polynom_codegen.cpp)
TARGET_INCLUDE_DIRECTORIES(polynom_codegen PRIVATE ${CppADCodeGen_INCLUDE_DIR})
