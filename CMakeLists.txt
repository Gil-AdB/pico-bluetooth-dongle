# CMakeLists.txt
cmake_minimum_required(VERSION 3.13)

# 1. First, include the SDK to make its functions available.
set(PICO_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/pico-sdk)

include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

# 2. Define the project name and languages. This is what the warning asked for.
project(pico_bluetooth_dongle C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# 3. Set the target board.
set(PICO_BOARD pico2_w)


#set(PICO_CYW43_ARCH_HEADER pico/cyw43_arch/arch_threaded.h)

# 4. Now, initialize the Pico SDK.
pico_sdk_init()

# 5. Add the source file to the build target.
add_executable(${PROJECT_NAME}
		src/main.c
		${PICO_SDK_PATH}/lib/tinyusb/src/class/bth/bth_device.c
)

# 6. Enable the CYW43 wireless driver support.
#pico_enable_cyw43_arch_default()

target_include_directories(${PROJECT_NAME} PRIVATE
		${PICO_SDK_PATH}/src/rp2_common/pico_cyw43_driver/include
		${PICO_SDK_PATH}/src/rp2_common/pico_cyw43_arch/include
		${PICO_SDK_PATH}/lib/lwip/src/include
		${PICO_SDK_PATH}/src/rp2_common/pico_lwip/include
		${PICO_SDK_PATH}/src/rp2_common/pico_async_context/include
		${PICO_SDK_PATH}/lib/btstack/platform/pico/
		${PICO_SDK_PATH}/lib/tinyusb/src/class/bth/
)

target_include_directories(${PROJECT_NAME} PRIVATE .)
target_sources(${PROJECT_NAME} PRIVATE
  src/main.c
  src/log_buffer.c
)
# 7. Link all necessary libraries to your executable.
target_link_libraries(${PROJECT_NAME}
		pico_stdlib
		pico_async_context_base
		pico_async_context_poll
		pico_async_context_threadsafe_background
		pico_cyw43_arch_threadsafe_background
		pico_btstack_ble
		pico_btstack_cyw43
		tinyusb_device
		tinyusb_board
)

# 8. Add project-specific compile definitions.
target_compile_definitions(${PROJECT_NAME} PRIVATE
		PICO_CYW43_ARCH_BLUETOOTH_ENABLED=1
		TUSB_CONFIG_FILE="tusb_config.h"
		PICO_CYW43_SUPPORTED
		ENABLE_BLE=1
		CYW43_LWIP=0
		PICO_STDIO_USB=0
		PICO_STDIO_UART=1
)

# 9. Ensure the final .uf2 file is generated for flashing.
pico_add_extra_outputs(${PROJECT_NAME})