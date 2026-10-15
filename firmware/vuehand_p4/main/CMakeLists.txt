set(VH_SERVO_TRANSPORT "uart" CACHE STRING
    "vuehand servo bus transport: 'uart' (default, raw GPIO/UART pins) or \
'usb' (USB-host VCP adapter e.g. URT-2/U2D2 -- build with \
`idf.py -D VH_SERVO_TRANSPORT=usb build`)")

if(VH_SERVO_TRANSPORT STREQUAL "usb")
    set(vh_transport_src "transport_usb.cpp")
else()
    set(vh_transport_src "transport_uart.c")
endif()

set(VH_HAND_SIDE "right" CACHE STRING
    "which ORCA hand this board drives: 'right' (default) or 'left'. Selects VH_RIGHT_MAP / VH_LEFT_MAP in joints_table.h at boot -- build with `idf.py -D VH_HAND_SIDE=left build`. Cache variable: it sticks for the build tree.")

if(VH_HAND_SIDE STREQUAL "left")
    set(vh_hand_right 0)
elseif(VH_HAND_SIDE STREQUAL "right")
    set(vh_hand_right 1)
else()
    message(FATAL_ERROR "VH_HAND_SIDE must be 'left' or 'right', got '${VH_HAND_SIDE}'")
endif()

# uart transport pins (main.c VH_SERVO_*). RTS -1 = no direction pin: the
# adapter switches the half-duplex line itself (Waveshare Bus Servo Adapter (A)).
set(VH_SERVO_TX "20" CACHE STRING "uart transport: P4 GPIO driving the servo bus TX")
set(VH_SERVO_RX "21" CACHE STRING "uart transport: P4 GPIO reading the servo bus RX")
set(VH_SERVO_RTS "8" CACHE STRING "uart transport: direction GPIO for an external driver, -1 for none")

idf_component_register(
    SRCS "main.c" "feetech.c" "infer.c" "jsonish.c" "motorcal.c" "servo.c" "${vh_transport_src}"
    INCLUDE_DIRS "."
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE VH_HAND_RIGHT=${vh_hand_right}
    VH_SERVO_TX=${VH_SERVO_TX} VH_SERVO_RX=${VH_SERVO_RX} VH_SERVO_RTS=${VH_SERVO_RTS})
