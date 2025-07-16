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
    "src/demod/dvbt/*.cpp"
)

# Set up compiler flags for SDR++ module system
if(NOT DEFINED SDRPP_MODULE_COMPILER_FLAGS)
    if(${CMAKE_BUILD_TYPE} MATCHES "Debug")
        # Debug Flags
        if(MSVC)
            set(SDRPP_MODULE_COMPILER_FLAGS /std:c++17 /EHsc)
        elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
            set(SDRPP_MODULE_COMPILER_FLAGS -g -Og -std=c++17 -Wno-unused-command-line-argument -undefined dynamic_lookup)
        else()
            set(SDRPP_MODULE_COMPILER_FLAGS -g -Og -std=c++17)
        endif()
    else()
        # Release Flags
        if(MSVC)
            set(SDRPP_MODULE_COMPILER_FLAGS /O2 /Ob2 /std:c++17 /EHsc)
        elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
            set(SDRPP_MODULE_COMPILER_FLAGS -O3 -std=c++17 -Wno-unused-command-line-argument -undefined dynamic_lookup)
        else()
            set(SDRPP_MODULE_COMPILER_FLAGS -O3 -std=c++17)
        endif()
    endif()
endif()

# Set up core root for SDR++ module system
if(NOT DEFINED SDRPP_CORE_ROOT)
    set(SDRPP_CORE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../core/src")
endif()

# Use SDR++ module system or fallback
if(DEFINED SDRPP_MODULE_CMAKE)
    message("Using provided SDRPP_MODULE_CMAKE: ${SDRPP_MODULE_CMAKE}")
    include(${SDRPP_MODULE_CMAKE})
else()
    message("SDRPP_MODULE_CMAKE not defined, looking for alternatives...")
    # Fallback: try to find the module cmake in common locations (skip the problematic hack file)
    set(SDRPP_MODULE_CMAKE_CANDIDATES
        "${CMAKE_CURRENT_LIST_DIR}/sdrpp_module.cmake"
        "${CMAKE_CURRENT_LIST_DIR}/../sdrpp_module.cmake"
        "${CMAKE_CURRENT_LIST_DIR}/../../sdrpp_module.cmake"
    )
    
    foreach(candidate ${SDRPP_MODULE_CMAKE_CANDIDATES})
        if(EXISTS ${candidate})
            set(SDRPP_MODULE_CMAKE ${candidate})
            message("Found SDRPP module cmake at: ${candidate}")
            break()
        endif()
    endforeach()
    
    if(DEFINED SDRPP_MODULE_CMAKE)
        include(${SDRPP_MODULE_CMAKE})
    else()
        message("No SDRPP module cmake found, using standalone build")
        # Basic module definition for standalone builds
        add_library(${PROJECT_NAME} SHARED ${SRC})
        set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")
        
        # Set compiler flags
        if(WIN32 AND MSVC)
            target_compile_options(${PROJECT_NAME} PRIVATE /std:c++17 /EHsc)
        else()
            target_compile_options(${PROJECT_NAME} PRIVATE -std=c++17)
        endif()
        
        # Set install target
        install(TARGETS ${PROJECT_NAME} DESTINATION lib/sdrpp/plugins)
    endif()
endif()

# Include directories
target_include_directories(dvbs_demodulator PRIVATE "src/" "src/demod")

# Find and link FFTW3 for DVB-T demodulator
if(WIN32)
    # On Windows, try to find FFTW3 using find_library
    find_library(FFTW3F_LIBRARIES NAMES fftw3f libfftw3f-3)
    if(FFTW3F_LIBRARIES)
        target_link_libraries(dvbs_demodulator PRIVATE ${FFTW3F_LIBRARIES})
        message("Found FFTW3F library on Windows: ${FFTW3F_LIBRARIES}")
    else()
        message(WARNING "FFTW3F not found on Windows - DVB-T demodulator may not work properly")
        # Try to find in common Windows locations
        find_library(FFTW3F_LIBRARIES NAMES fftw3f
            PATHS
            "C:/Program Files/fftw"
            "C:/Program Files (x86)/fftw"
            "C:/fftw"
            "C:/vcpkg/installed/x64-windows/lib"
            "C:/vcpkg/installed/x86-windows/lib"
            PATH_SUFFIXES lib
        )
        if(FFTW3F_LIBRARIES)
            target_link_libraries(dvbs_demodulator PRIVATE ${FFTW3F_LIBRARIES})
            message("Found FFTW3F in Windows paths: ${FFTW3F_LIBRARIES}")
        endif()
    endif()
else()
    # On Unix-like systems, try to use pkg-config first
    find_package(PkgConfig QUIET)
    if(PKG_CONFIG_FOUND)
        pkg_check_modules(FFTW3F QUIET fftw3f)
        
        if(FFTW3F_FOUND)
            target_include_directories(dvbs_demodulator PRIVATE ${FFTW3F_INCLUDE_DIRS})
            target_link_libraries(dvbs_demodulator PRIVATE ${FFTW3F_LIBRARIES})
            target_compile_definitions(dvbs_demodulator PRIVATE ${FFTW3F_CFLAGS_OTHER})
            message("Found FFTW3F via pkg-config: ${FFTW3F_LIBRARIES}")
        else()
            # Fallback to find_library
            find_library(FFTW3F_LIBRARIES NAMES fftw3f)
            if(FFTW3F_LIBRARIES)
                target_link_libraries(dvbs_demodulator PRIVATE ${FFTW3F_LIBRARIES})
                message("Found FFTW3F library: ${FFTW3F_LIBRARIES}")
            else()
                message(WARNING "FFTW3F not found - DVB-T demodulator may not work properly")
            endif()
        endif()
    else()
        # No pkg-config, try find_library
        find_library(FFTW3F_LIBRARIES NAMES fftw3f)
        if(FFTW3F_LIBRARIES)
            target_link_libraries(dvbs_demodulator PRIVATE ${FFTW3F_LIBRARIES})
            message("Found FFTW3F library: ${FFTW3F_LIBRARIES}")
        else()
            message(WARNING "FFTW3F not found - DVB-T demodulator may not work properly")
        endif()
    endif()
endif()

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
