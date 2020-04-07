add_library(gpio EXCLUDE_FROM_ALL
    LinuxGpio.cpp
)

target_include_directories(gpio
    PUBLIC .
)

conan_add_remote(NAME kubasejdak URL https://api.bintray.com/conan/kubasejdak/public-conan)
conan_get(libgpiod/1.2.1@kubasejdak/stable)
find_package(libgpiod REQUIRED)

target_link_libraries(gpio
    PUBLIC hal-public hal-private libgpiod::libgpiod utils
)
