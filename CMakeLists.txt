cmake_minimum_required(VERSION 3.17)
project(sim960)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
find_package(Boost
		REQUIRED
		COMPONENTS program_options)

add_library(core960
		include/Entry255.def
		include/Opcodes.def
		include/BinaryOperations.h
		include/Core.h
		include/IAC.h
		include/Types.h
		src/Core.cc
		src/IAC.cc
		src/Interrupts.cc
		src/Faults.cc
		src/ProtectedArchitecture.cc
		src/NumericsArchitecture.cc
		src/StartupTests.cc
		src/MemoryArchitecture.cc
		src/CallOperations.cc
		src/RegisterOperations.cc
		include/Disassembly.h
		src/Disasembly.cc
		include/Platform.h
		src/Interpreter.cc
	)
add_library(nativecore
		src/NativeCore.cc)

add_executable(sim5
		cmd/sim5/main.cc
		)
add_executable(mmu_test
		cmd/mmu_test/main.cc)
target_link_libraries(sim5
		core960
		nativecore
		Boost::program_options)
target_link_libraries(mmu_test
		core960
		nativecore
		)
install(TARGETS sim5 DESTINATION bin)

target_include_directories(core960 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(mmu_test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(nativecore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
