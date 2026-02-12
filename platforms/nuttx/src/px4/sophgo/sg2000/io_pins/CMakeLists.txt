############################################################################
#
#   Copyright (c) 2026 Stalya Inc. All rights reserved.
#
############################################################################

px4_add_library(arch_io_pins
	pwm_servo.c
)

target_link_libraries(arch_io_pins PRIVATE drivers_board)
