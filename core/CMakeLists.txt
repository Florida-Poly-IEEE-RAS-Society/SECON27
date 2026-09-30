add_library(core STATIC
    src/math/vector.c
    src/math/matrix.c
)

target_include_directories(core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# libm for sqrtf
find_library(MATH_LIBRARY m)
if(MATH_LIBRARY)
    target_link_libraries(core PUBLIC ${MATH_LIBRARY})
endif()
