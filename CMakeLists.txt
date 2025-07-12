cmake_minimum_required(VERSION 3.13)
project(dvbs_demodulator)

# Include cmake modules from the local directory
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Collect all source files
file(GLOB_RECURSE SRC 
    "src/*.cpp"
    "src/*.c"
    "src/demod/common/codings/*.cpp"
    "src/demod/common/correct/*.cpp"
    "src/demod/common/correct/reed-solomon/*.c"
    "src/demod/common/dsp/demod/*.cpp"
    "src/demod/common/*.cpp"
    "src/demod/dvbs2/codings/*.cpp"
    "src/demod/dvbs2/codings/bch/*.cpp"
    "src/demod/dvbs2/codings/xdsopl-ldpc-pabr/*.cc"
    "src/demod/dvbs2/*.cpp"
    "src/demod/dvbs/*.cpp"
    "src/demod/dvbs/viterbi/*.cpp"
)

# Use SDR++ module system
include(${SDRPP_MODULE_CMAKE})

# Include directories
target_include_directories(dvbs_demodulator PRIVATE "src/" "src/demod")

# Add Windows-specific definitions
if(WIN32)
    target_compile_definitions(dvbs_demodulator PRIVATE NOMINMAX)
endif()

# SSE41 optimization for Windows
if(NOT ANDROID)
    # Try to find SSE41 support
    if(WIN32 AND MSVC)
        # Enable SSE4.1 for MSVC on Windows
        target_compile_options(dvbs_demodulator PRIVATE /arch:SSE2)
        message("Enabled SSE optimizations for DVB-S demodulator on Windows")
    elseif(UNIX)
        # Enable SSE4.1 for GCC/Clang on Unix
        include(CheckCXXCompilerFlag)
        check_cxx_compiler_flag("-msse4.1" COMPILER_SUPPORTS_SSE41)
        if(COMPILER_SUPPORTS_SSE41)
            target_compile_options(dvbs_demodulator PRIVATE -msse4.1)
            message("Enabled SSE4.1 optimizations for DVB-S demodulator")
        endif()
    endif()
endif()
