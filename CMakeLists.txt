cmake_minimum_required(VERSION 3.24)
set(APP_NAME usbaudio)
project(${APP_NAME} C)

set(CMAKE_C_STANDARD 17)

find_package(PkgConfig REQUIRED)

pkg_check_modules(SDL2 REQUIRED sdl2)

pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
link_directories(${SDL2_LIBRARY_DIRS} ${LIBUSB_LIBRARY_DIRS})
add_compile_definitions(USE_LIBUSB)

add_executable(${APP_NAME} main.c audio.c)

target_link_libraries(${APP_NAME} ${SDL2_LIBRARIES} ${LIBUSB_LIBRARIES})
target_include_directories(${APP_NAME} PUBLIC ${SDL2_INCLUDE_DIRS} ${LIBUSB_INCLUDE_DIRS})
target_compile_options(${APP_NAME} PUBLIC ${SDL2_CFLAGS_OTHER} ${LIBUSB_CFLAGS_OTHER})
