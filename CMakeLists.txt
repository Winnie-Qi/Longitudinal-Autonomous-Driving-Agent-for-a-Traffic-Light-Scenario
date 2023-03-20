cmake_minimum_required(VERSION 3.5)
project(basic_agent_ST)

set(CMAKE_CXX_STANDARD 11)
# If you are on mac x86
#set(CMAKE_OSX_ARCHITECTURES x86_64)

include(lib/basic.cmake)

include_directories(
        matlab/codegen/lib/primitives/
        matlab/
        lib
        log
)

link_directories(
        lib
        lib/win_visual_studio
)

# ------------------------------- basic agent ------------------------------------
add_executable(basic_agent
        matlab/codegen/lib/primitives/rt_nonfinite.c
        matlab/codegen/lib/primitives/primitives_rtwutil.c
        matlab/codegen/lib/primitives/pass_primitive.c
        matlab/codegen/lib/primitives/pass_primitive_j0.c
        matlab/codegen/lib/primitives/stop_primitive.c
        matlab/codegen/lib/primitives/stop_primitive_j0.c

        log/screen_print.c
        log/logvars.cc
        starting_point.cc
    )


target_link_libraries(basic_agent agent_lib ${lib_ws32})

target_compile_options(basic_agent
        PUBLIC -DNOT_INLINED -DSERVER
        )

set_target_properties(basic_agent PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${AGENT_FOLDER})
# ------------------------------------------------------------------------------------
