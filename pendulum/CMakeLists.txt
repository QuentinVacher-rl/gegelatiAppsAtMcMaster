cmake_minimum_required(VERSION 3.12.4)

# *******************************************
# ************* CMake Content ***************
# *******************************************
# This CMake create a workspace containing the following projects
# 
# Programs
#  - pendulum

set (PROJECT_NAME pendulum)

project(${PROJECT_NAME})

option( TESTING "Is the program compiled for tests only (without display)." OFF)

# Add definition for relative path into project
add_definitions( -DPROJECT_ROOT_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

# Disable C and C++ compiler extensions.
# C/CXX_EXTENSIONS are ON by default to allow the compilers to use extended
# variants of the C/CXX language.
# However, this could expose cross-platform bugs in user code or in the headers
# of third-party dependencies and thus it is strongly suggested to turn
# extensions off.
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set Release as default build type if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build (Debug or Release)" FORCE)
endif()

if(NOT ${CMAKE_GENERATOR} MATCHES "Visual Studio.*")

	# Link with pthread
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

	# Debug or release
	if(CMAKE_BUILD_TYPE MATCHES "Debug")
			MESSAGE("Generate Debug project")
			set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/Debug)
			set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -pg -Wall")
	else()
			MESSAGE("Generate Release project")
			set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/Release)
			set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")
	endif()
	#add libmath during non visual studio builds
	set(CMAKE_EXTRA_LIB m)
else()
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
	add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

# Add definitions for testing purposes
if(${TESTING})
	MESSAGE("Testing mode")
	add_definitions(-DNO_CONSOLE_CONTROL -DNB_GENERATIONS=2)
endif()

# *******************************************
# *********** GEGELATI LIBRARY **************
# *******************************************

if(WIN32)
	set(LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
    # find the gegelatilib-x.y.z folder in the lib directory.
	file(GLOB GEGELATI_ROOT_DIR "${LIBS_DIR}/gegelatilib-[\\.|0-9]*")
	set(ENV{GEGELATI_DIR} ${GEGELATI_ROOT_DIR})
endif()
find_package(GEGELATI)


if (WIN32)
	file(GLOB
		GEGELATI_DLL
		${GEGELATI_ROOT_DIR}/bin/*.dll
	)

	MESSAGE("Copy GEGELATI DLLs into ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
	file(COPY ${GEGELATI_DLL} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
	if(${CMAKE_GENERATOR} MATCHES "Visual Studio.*")
		file(COPY ${GEGELATI_DLL} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Debug)
		file(COPY ${GEGELATI_DLL} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Release)
	endif()
endif()

if(NOT ${TESTING})
	# *******************************************
	# ************** SDL2 LIBRARY ***************
	# *******************************************
	# Set variables before finding SDL Library
	# Helps to find where SDL libraries are saved (win32 only)
	set(LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
	set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/lib/cmake_modules/")
	
	if(WIN32)
		# find the SDL2-x.y.z folder in the lib directory.
		file(GLOB SDL2DIR "${LIBS_DIR}/SDL2-[\\.|0-9]*")
		set(ENV{SDL2DIR} ${SDL2DIR})
	endif()
	
	Find_Package (SDL2 REQUIRED )
	
	if(NOT SDL2_FOUND)
	MESSAGE(FATAL_ERROR "SDL not found !")
	endif()
	
	if (WIN32)
		file(GLOB
			SDL2_DLL
			${SDL2DIR}/bin/*.dll
			${SDL2DIR}/lib/*.dll
		)
	
		MESSAGE("Copy SDL DLLs into ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
		if(NOT ${CMAKE_GENERATOR} MATCHES "Visual Studio.*")
			file(COPY ${SDL2_DLL} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
		else()
			file(COPY ${SDL2_DLL} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Debug)
			file(COPY ${SDL2_DLL} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Release)
		endif()
	endif()
	
	# *******************************************
	# *********** SDL2_image LIBRARY ************
	# *******************************************
	# Set variables before finding SDL_image Library
	# Helps to find where SDL libraries are saved (win32 only)
	
	if(WIN32)
		# find the SDL2_image-x.y.z folder in the lib directory.
		file(GLOB SDL2IMAGEDIR "${LIBS_DIR}/SDL2_image-[\\.|0-9]*")
		set(ENV{SDL2IMAGEDIR} ${SDL2IMAGEDIR})
	endif()
	
	Find_Package (SDL2_image REQUIRED )
	
	if(NOT SDL2IMAGE_FOUND)
	MESSAGE(FATAL_ERROR "SDL Image not found !")
	endif()
	
	if (WIN32)
		file(GLOB
			SDL2_IMAGE_DLL
			${SDL2IMAGEDIR}/bin/*.dll
			${SDL2IMAGEDIR}/lib/*.dll
		)
	
		MESSAGE("Copy SDL Image DLLs into ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
		if(NOT ${CMAKE_GENERATOR} MATCHES "Visual Studio.*")
			file(COPY ${SDL2_IMAGE_DLL} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
		else()
			file(COPY ${SDL2_IMAGE_DLL} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Debug)
			file(COPY ${SDL2_IMAGE_DLL} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Release)
		endif()
	endif()
	
	# *******************************************
	# ************ SDL2TTF LIBRARY **************
	# ******************************************
	# Set variables before finding SDL2TTF Library
	# Helps to find where SDL2TTF libraries are saved (win32 only)
	if(WIN32)
		# find the SDL2_ttf-x.y.z folder in the lib directory.
		file(GLOB SDL2TTFDIR "${LIBS_DIR}/SDL2_ttf-[\\.|0-9]*")
		set(ENV{SDL2TTFDIR} ${SDL2TTFDIR})
	endif()
	
	
	Find_Package (SDL2TTF REQUIRED )
	
	if(NOT SDL2TTF_FOUND)
	MESSAGE(FATAL_ERROR "SDL2TTF not found !")
	endif()
	
	if (WIN32)
		file(GLOB
			SDL2TTF_DLL
			${SDL2TTFDIR}/bin/*.dll
			${SDL2TTFDIR}/lib/*.dll
		)
	
		MESSAGE("Copy SDL2_ttf DLLs into ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
		if(NOT ${CMAKE_GENERATOR} MATCHES "Visual Studio.*")
			file(COPY ${SDL2TTF_DLL} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
		else()
			file(COPY ${SDL2TTF_DLL} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Debug)
			file(COPY ${SDL2TTF_DLL} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Release)
		endif()
	endif()
endif()
	
# *******************************************
# ************** Executable  ****************
# *******************************************

# Executable to learn the TPG
file(GLOB_RECURSE
	pendulum_files
	./src/Learn/*.cpp
	./src/Learn/*.h
	./params.json
)

include_directories(${GEGELATI_INCLUDE_DIRS}  ${SDL2_INCLUDE_DIR} ${SDL2IMAGE_INCLUDE_DIR} ${SDL2TTF_INCLUDE_DIR})
add_executable(${PROJECT_NAME} ${pendulum_files})
target_link_libraries(${PROJECT_NAME} ${GEGELATI_LIBRARIES}  ${SDL2_LIBRARY} ${SDL2IMAGE_LIBRARY} ${SDL2TTF_LIBRARY})
target_compile_definitions(${PROJECT_NAME} PRIVATE ROOT_DIR="${CMAKE_SOURCE_DIR}")

# Code Gen example with the TPG store in the file pendulum/src/CodeGen/Pendulum_out_best.dot

# Create the target that will generate the C code of the TPG
set(TARGET_CodeGen ${PROJECT_NAME}CodeGenCompile)
add_executable(${TARGET_CodeGen} src/Learn/instructions.cpp src/CodeGen/mainCodeGenCompile.cpp)
target_link_libraries(${TARGET_CodeGen} ${GEGELATI_LIBRARIES})
target_compile_definitions(${TARGET_CodeGen} PRIVATE ROOT_DIR="${CMAKE_SOURCE_DIR}")
if (WIN32)
  set_target_properties(${TARGET_CodeGen} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR})
  set_target_properties(${TARGET_CodeGen} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR})
endif()

# set and create the source directory where file generated by the codegen are saved.
# set a variable that groups all the codeGen files
set(SRC_CODEGEN ${CMAKE_CURRENT_BINARY_DIR}/src/)
set(SRC ./src/CodeGen)
file(MAKE_DIRECTORY ${SRC_CODEGEN})
include_directories(${SRC_CODEGEN} ${SRC})
set(CODEGEN ${SRC_CODEGEN}/${PROJECT_NAME}.c ${SRC_CODEGEN}/${PROJECT_NAME}_program.c ${SRC_CODEGEN}/${PROJECT_NAME}.h ${SRC_CODEGEN}/${PROJECT_NAME}_program.h)

# set codeGen source file as generated
set_source_files_properties(${CODEGEN} PROPERTIES GENERATED TRUE)

# wrap generation of source file in a custom command + custom target
add_custom_command(OUTPUT ${CODEGEN} COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TARGET_CodeGen})
set(ExecCodeGen ${PROJECT_NAME}ExecCodeGen)
add_custom_target(${ExecCodeGen} DEPENDS ${CODEGEN})
add_dependencies(${ExecCodeGen} ${TARGET_CodeGen})

# set target name for the executable that use the codeGen files to do an inference
set(TARGET ${PROJECT_NAME}InferenceCodeGen)
# create binary that need the generated file
set(inference_codegen_files ${SRC}/mainCodeGenInference.cpp ./src/Learn/instructions.cpp ./src/Learn/pendulum.cpp ${CODEGEN})

add_executable(${TARGET} ${inference_codegen_files})
target_link_libraries(${TARGET} ${GEGELATI_LIBRARIES} ${SDL2_LIBRARY} ${SDL2IMAGE_LIBRARY} ${SDL2TTF_LIBRARY})
target_include_directories(${TARGET} BEFORE PUBLIC ${SRC_CODEGEN}) 
target_compile_definitions(${TARGET} PRIVATE ROOT_DIR="${CMAKE_SOURCE_DIR}")
# set the custom target that generate the source file as a dependency of the target
add_dependencies(${TARGET} ${ExecCodeGen})

# Create a project that does the inference from the dot 
set(TARGET_TPGInference ${PROJECT_NAME}TPGInference)
set(TARGET_TPGInference_files ${pendulum_files} ${SRC}/mainTPGInference.cpp)
list(REMOVE_ITEM TARGET_TPGInference_files "${CMAKE_CURRENT_SOURCE_DIR}/./src/Learn/main.cpp")
add_executable(${TARGET_TPGInference} ${TARGET_TPGInference_files})
target_compile_definitions(${TARGET_TPGInference} PRIVATE ROOT_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(${TARGET_TPGInference} ${GEGELATI_LIBRARIES}  ${SDL2_LIBRARY} ${SDL2IMAGE_LIBRARY} ${SDL2TTF_LIBRARY})
