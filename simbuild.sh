#!/bin/sh

export PX4_GZ_DEBUG=1
export PX4_GZ_WORLD=default
make px4_sitl gz_babyshark_vtol
#make px4_sitl gz_standard_vtol
#make px4_sitl gz_alti_transition_quad
