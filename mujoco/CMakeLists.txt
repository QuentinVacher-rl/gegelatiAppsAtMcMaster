cmake_minimum_required(VERSION 3.12.4)

# *******************************************
# ************* CMake Content ***************
# *******************************************
# This CMake create a workspace containing the following projects
# 
# Programs
#  - mujoco

set (PROJECT_NAME mujoco)

set (RENDER_NAME renderMujoco)

project(${PROJECT_NAME})

option( RENDERING "Is the program compiled will render (with display)." ON)

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

# *******************************************
# *********** GEGELATI LIBRARY **************
# *******************************************

# Set the module path to find GEGELATI
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/gegelati/bin/")
set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/gegelati/bin" ${CMAKE_PREFIX_PATH})
find_package(GEGELATI REQUIRED)

# Include and link GEGELATI
if(GEGELATI_FOUND)
    message(STATUS "GEGELATI version ${GEGELATI_VERSION} found.")
    include_directories(${GEGELATI_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "GEGELATI not found!")
endif()

# *******************************************
# ************ MUJOCO LIBRARY ***************
# *******************************************


# Ajouter le chemin vers les fichiers d'en-tête de MuJoCo
include_directories(${CMAKE_SOURCE_DIR}/lib/mujoco/include)

# Spécifier le chemin vers les bibliothèques MuJoCo
link_directories(${CMAKE_SOURCE_DIR}/lib/mujoco/bin)

find_package(GLEW REQUIRED)
if(${RENDERING})
	MESSAGE("Render mode")
	find_package(OpenGL REQUIRED)
	
	find_package(X11 REQUIRED)
endif()



# *******************************************
# ************** Executable  ****************
# *******************************************

# Executable to learn the TPG
file(GLOB_RECURSE
	mujoco_files
	${CMAKE_SOURCE_DIR}/src/*.cpp
	${CMAKE_SOURCE_DIR}/src/*.h
	${CMAKE_SOURCE_DIR}/params/params_0.json
)

list(REMOVE_ITEM
	mujoco_files
	${CMAKE_SOURCE_DIR}/src/mainRender.cpp
	${CMAKE_SOURCE_DIR}/src/mainRender.h)

include_directories(${GEGELATI_INCLUDE_DIRS}  ${SDL2_INCLUDE_DIR} ${SDL2IMAGE_INCLUDE_DIR} ${SDL2TTF_INCLUDE_DIR} mujoco210)
add_executable(${PROJECT_NAME} ${mujoco_files})
target_link_libraries(${PROJECT_NAME} ${GEGELATI_LIBRARIES}  ${SDL2_LIBRARY} ${SDL2IMAGE_LIBRARY} ${SDL2TTF_LIBRARY} mujoco210 ${GLEW_LIBRARIES} ${OPENGL_LIBRARIES})
target_compile_definitions(${PROJECT_NAME} PRIVATE ROOT_DIR="${CMAKE_SOURCE_DIR}")

if(${RENDERING})

	list(REMOVE_ITEM
		mujoco_files
		${CMAKE_SOURCE_DIR}/src/main.cpp)


	# Liste des fichiers source
	list(APPEND mujoco_files
		${CMAKE_SOURCE_DIR}/src/mainRender.cpp
		${CMAKE_SOURCE_DIR}/src/mainRender.h
		${CMAKE_SOURCE_DIR}/lib/stb_image_write.h
	)

	# Inclusion des répertoires
	include_directories(${GEGELATI_INCLUDE_DIRS}  
						${SDL2_INCLUDE_DIR} 
						${SDL2IMAGE_INCLUDE_DIR} 
						${SDL2TTF_INCLUDE_DIR} 
						mujoco210 
						glfw3 
						${CMAKE_SOURCE_DIR}/lib)  # Assurez-vous que le dossier lib est inclus

	# Création de l'exécutable
	add_executable(${RENDER_NAME} ${mujoco_files})

	# Lien des bibliothèques
	target_link_libraries(${RENDER_NAME} 
		mujoco210 
		glfw3 
		${GLEW_LIBRARIES} 
		${OPENGL_LIBRARIES} 
		${GEGELATI_LIBRARIES}  
		${SDL2_LIBRARY} 
		${SDL2IMAGE_LIBRARY} 
		${SDL2TTF_LIBRARY} 
		X11 
		Xrandr 
		Xxf86vm
		Xinerama
		Xcursor
	)

	# Définition d'une macro avec le répertoire racine du projet
	target_compile_definitions(${RENDER_NAME} PRIVATE ROOT_DIR="${CMAKE_SOURCE_DIR}")
endif()
