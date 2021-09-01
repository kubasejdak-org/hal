add_library(sbc-raspberrypi-public INTERFACE)
add_library(sbc-raspberrypi-private EXCLUDE_FROM_ALL
    BoardInit.cpp
)
add_library(sbc-raspberrypi-config INTERFACE)

target_include_directories(sbc-raspberrypi-public
    INTERFACE include
)

target_include_directories(sbc-raspberrypi-config
    INTERFACE config
)

target_link_libraries(sbc-raspberrypi-public
    INTERFACE hal::interfaces
)

target_link_libraries(sbc-raspberrypi-private
    PUBLIC sbc-raspberrypi-public hal::interfaces
    PRIVATE hal::board-interface hal::gpio hal::spi hal::i2c
)

target_link_libraries(sbc-raspberrypi-config
    INTERFACE utils::types
)
