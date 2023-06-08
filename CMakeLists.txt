cmake_minimum_required(VERSION 3.17)
project(sim960)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
find_package(Boost
	REQUIRED
	COMPONENTS program_options)

add_library(core960
		include/BinaryOperations.h
		include/Core.h
		include/IAC.h
		include/Types.h
		src/Core.cc
		src/IAC.cc src/Interrupts.cc src/Faults.cc)

add_executable(sim5 
	src/main.cc
	src/NativeCore.cc)
target_link_libraries(sim5 
	core960
	Boost::program_options)
install(TARGETS sim5 DESTINATION bin)

target_include_directories(core960 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
