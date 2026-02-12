/****************************************************************************
 *
 *   Copyright (C) 2025 Stalya Inc. All rights reserved.
 *
 *   Author: Volvox <volvox@stalya.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4io_serial.cpp
 *
 * Serial interface for PX4IO on MPFS
 */

#include <syslog.h>

#include <px4_arch/px4io_serial.h>
#include <termios.h>
#include <debug.h>


uint8_t ArchPX4IOSerial::_io_buffer_storage[sizeof(IOPacket)];

ArchPX4IOSerial::ArchPX4IOSerial() :
	_current_packet(nullptr),
	_completion_semaphore(SEM_INITIALIZER(0))
{
}

ArchPX4IOSerial::~ArchPX4IOSerial()
{
	/* detach our interrupt handler */

	up_disable_irq(PX4IO_SERIAL_VECTOR);
	irq_detach(PX4IO_SERIAL_VECTOR);

	/* and kill our semaphores */

	px4_sem_destroy(&_completion_semaphore);
}

int ArchPX4IOSerial::init_uart()
{
	/* Attach serial interrupt handler */

	irq_attach(PX4IO_SERIAL_VECTOR, _interrupt, this);
	up_enable_irq(PX4IO_SERIAL_VECTOR);

	/* Create semaphores */

	px4_sem_init(&_completion_semaphore, 0, 0);

	/* _completion_semaphore use case is a signal */

	px4_sem_setprotocol(&_completion_semaphore, SEM_PRIO_NONE);

	return 0;
}

int
ArchPX4IOSerial::init()
{
	/* initialize base implementation */
	int r = PX4IO_serial::init((IOPacket *)&_io_buffer_storage[0]);

	if (r != 0) {
		return r;
	}

	r = init_uart();

	return r;
}

int
ArchPX4IOSerial::ioctl(unsigned operation, unsigned &arg)
{
	int ret = 0;

	switch (operation) {

	case 1:		/* XXX magic number - test operation */
		switch (arg) {
		case 0:
			break;

		case 1: {
				unsigned fails = 0;

				for (unsigned count = 0;; count++) {
					uint16_t value = count & 0xffff;

					if (write((PX4IO_PAGE_TEST << 8) | PX4IO_P_TEST_LED, &value, 1) != 0) {
						fails++;
					}

					if (count >= 5000) {
						syslog(LOG_INFO, "==== test 1 : %u failures ====\n", fails);
						perf_print_counter(_pc_txns);
						perf_print_counter(_pc_retries);
						perf_print_counter(_pc_timeouts);
						perf_print_counter(_pc_crcerrs);
						perf_print_counter(_pc_protoerrs);
						perf_print_counter(_pc_uerrs);
						count = 0;
					}
				}
			}
			break;

		case 2:
			syslog(LOG_INFO, "test 2\n");
			break;

		default:
			break;
		}

		break;

	default:
		ret = -1;
		break;
	}

	return ret;
}

int
ArchPX4IOSerial::_bus_exchange(IOPacket *_packet)
{
	return OK;
}

int
ArchPX4IOSerial::_interrupt(int irq, void *context, void *arg)
{
	return 0;
}

void
ArchPX4IOSerial::_do_interrupt()
{
}
