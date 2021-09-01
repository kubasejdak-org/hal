add_library(hal-board-public INTERFACE)
add_library(hal::board-public ALIAS hal-board-public)
add_library(hal-board-private INTERFACE)
add_library(hal::board-private ALIAS hal-board-private)
add_library(hal-board-config INTERFACE)
add_library(hal::board-config ALIAS hal-board-config)
add_library(hal-board-interface INTERFACE)
add_library(hal::board-interface ALIAS hal-board-interface)

target_include_directories(hal-board-interface
    INTERFACE include
)

foreach (BOARD_PATH IN LISTS HAL_BOARDS)
    add_subdirectory(${BOARD_PATH})

    string(REPLACE "/" "-" BOARD "${BOARD_PATH}")
    target_link_libraries(hal-board-public
        INTERFACE ${BOARD}-public
    )
    target_link_libraries(hal-board-private
        INTERFACE ${BOARD}-private
    )

    if (TARGET ${BOARD}-config)
        target_link_libraries(hal-board-config
            INTERFACE ${BOARD}-config
        )
    endif ()
endforeach ()
