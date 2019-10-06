CMAKE_ALT1 := /usr/local/bin/cmake
CMAKE_ALT2 := /Applications/CMake.app/Contents/bin/cmake
CMAKE := $(shell \
	which cmake 2>/dev/null || \
	([ -e ${CMAKE_ALT1} ] && echo "${CMAKE_ALT1}") || \
	([ -e ${CMAKE_ALT2} ] && echo "${CMAKE_ALT2}") \
	)

all: Release


Debug: build
	(cd build && ${CMAKE} -DCMAKE_BUILD_TYPE=$@ .. && make)

MinSizeRel: build
	(cd build && ${CMAKE} -DCMAKE_BUILD_TYPE=$@ .. && make)

Release: build
	(cd build && ${CMAKE} -DCMAKE_BUILD_TYPE=$@ .. && make)

RelWithDebugInfo: build
	(cd build && ${CMAKE} -DCMAKE_BUILD_TYPE=$@ .. && make)


build:
	mkdir -p build

clean:
	((cd build && make clean) 2>&- || true)

.PHONY: all Debug MinSizeRel Release RelWithDebugInfo clean
