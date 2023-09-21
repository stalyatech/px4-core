/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file TattuSocketCan.cpp
 * @author Jacob Dahl <dahl.jakejacob@gmail.com>
 *
 * Driver for the Tattu 12S 1600mAh Smart Battery connected over CAN.
 *
 * This driver simply decodes the CAN frames based on the specification
 * as provided in the Tattu datasheet DOC 001 REV D, which is highly
 * specific to the 12S 1600mAh battery. Other models of Tattu batteries
 * will NOT work with this driver in its current form.
 *
 */

#include "TattuSocketCan.hpp"

#include <debug.h>

#include <px4_platform_common/log.h>

extern orb_advert_t mavlink_log_pub;

TattuSocketCan::TattuSocketCan() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan)
{
}

TattuSocketCan::~TattuSocketCan()
{
}

int TattuSocketCan::OpenSocket(int bus)
{
	struct sockaddr_can addr;
	struct ifreq ifr;
	int ret = 0;

	/* Select interface and pins */

	switch (bus)
	{
	case 0:
		strlcpy(ifr.ifr_name, "can0", IFNAMSIZ);
		break;

	case 1:
		strlcpy(ifr.ifr_name, "can1", IFNAMSIZ);
		break;
	}

	/* Find network interface */

	ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
	if (!ifr.ifr_ifindex)
	{
		canerr("CAN%d : if_nametoindex failed\n", bus);
		return -1;
	}

	/* Open socket */

	if ((_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
	{
		canerr("CAN%d : Failed to open socket\n", bus);
		_sock = -1;
		return -1;
	}

	/* Bring up the interface */

	ifr.ifr_flags = IFF_UP;
	ret = ioctl(_sock, SIOCSIFFLAGS, (unsigned long)&ifr);
	if (ret < 0)
	{
		canerr("CAN%d : ioctl failed (can't set interface flags)\n", bus);
		close(_sock);
		_sock = -1;
		return -1;
	}

	/* Initialize sockaddr struct */

	memset(&addr, 0, sizeof(addr));
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	/* Disable default receive filter on this RAW socket
	*
	* This is obsolete as we do not read from the socket at all, but for this
	* reason we can remove the receive list in the kernel to save a little
	* (really very little!) CPU usage.
	*/

	setsockopt(_sock, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

	/* Bind socket and send the CAN frames */

	if (bind(_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
	{
		canerr("CAN%d : Failed to bind socket\n", bus);
		close(_sock);
		return -1;
	}

	/* Setup RX msg */
	memset(_recv_control, 0x00, sizeof(_recv_control));
	_recv_iov.iov_base = &_recv_frame;
	_recv_iov.iov_len = sizeof(struct can_frame);

	_recv_msg.msg_iov = &_recv_iov;
	_recv_msg.msg_iovlen = 1;
	_recv_msg.msg_control = &_recv_control;
	_recv_msg.msg_controllen = sizeof(_recv_control);
	_recv_cmsg = CMSG_FIRSTHDR(&_recv_msg);

	return 0;
}

void TattuSocketCan::CloseSocket()
{
	/* Close the socket */

	if (_sock != -1)
	{
		close(_sock);
		_sock = -1;
	}
}

void TattuSocketCan::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	if (!_initialized) {

		if (OpenSocket(0) < 0) {
			PX4_INFO("CAN0 : Failed to open socket\n");
			return;
		}

		_initialized = true;
	}

	uint8_t data[64] {};
	CanFrame received_frame{};
	received_frame.payload = &data;
	Tattu12SBatteryMessage tattu_message = {};

	while (receive(&received_frame) > 0) {

		// Find the start of a transferr
		if ((received_frame.payload_size == 8) && ((uint8_t *)received_frame.payload)[7] == TAIL_BYTE_START_OF_TRANSFER) {
		} else {
			continue;
		}

		// We have the start of a transfer
		size_t offset = 5;
		memcpy(&tattu_message, &(((uint8_t *)received_frame.payload)[2]), offset);

		while (receive(&received_frame) > 0) {

			size_t payload_size = received_frame.payload_size - 1;
			// TODO: add check to prevent buffer overflow from a corrupt 'payload_size' value
			// TODO: AND look for TAIL_BYTE_START_OF_TRANSFER to indicate end of transfer. Untested...
			memcpy(((char *)&tattu_message) + offset, received_frame.payload, payload_size);
			offset += payload_size;
		}

		battery_status_s battery_status = {};
		battery_status.timestamp = hrt_absolute_time();
		battery_status.connected = true;
		battery_status.cell_count = 12;

		battery_status.serial_number = tattu_message.manufacturer;
		battery_status.id = static_cast<uint8_t>(tattu_message.sku);

		battery_status.cycle_count = tattu_message.cycle_life;
		battery_status.state_of_health = static_cast<uint16_t>(tattu_message.health_status);

		battery_status.voltage_v = static_cast<float>(tattu_message.voltage) / 1000.0f;
		battery_status.voltage_filtered_v = static_cast<float>(tattu_message.voltage) / 1000.0f;
		battery_status.current_a = static_cast<float>(tattu_message.current) / 1000.0f;
		battery_status.current_filtered_a = static_cast<float>(tattu_message.current) / 1000.0f;
		battery_status.remaining = static_cast<float>(tattu_message.remaining_percent) / 100.0f;
		battery_status.temperature = static_cast<float>(tattu_message.temperature);
		battery_status.capacity = tattu_message.standard_capacity;
		battery_status.voltage_cell_v[0] = static_cast<float>(tattu_message.cell_1_voltage) / 1000.0f;
		battery_status.voltage_cell_v[1] = static_cast<float>(tattu_message.cell_2_voltage) / 1000.0f;
		battery_status.voltage_cell_v[2] = static_cast<float>(tattu_message.cell_3_voltage) / 1000.0f;
		battery_status.voltage_cell_v[3] = static_cast<float>(tattu_message.cell_4_voltage) / 1000.0f;
		battery_status.voltage_cell_v[4] = static_cast<float>(tattu_message.cell_5_voltage) / 1000.0f;
		battery_status.voltage_cell_v[5] = static_cast<float>(tattu_message.cell_6_voltage) / 1000.0f;
		battery_status.voltage_cell_v[6] = static_cast<float>(tattu_message.cell_7_voltage) / 1000.0f;
		battery_status.voltage_cell_v[7] = static_cast<float>(tattu_message.cell_8_voltage) / 1000.0f;
		battery_status.voltage_cell_v[8] = static_cast<float>(tattu_message.cell_9_voltage) / 1000.0f;
		battery_status.voltage_cell_v[9] = static_cast<float>(tattu_message.cell_10_voltage / 1000.0f);
		battery_status.voltage_cell_v[10] = static_cast<float>(tattu_message.cell_11_voltage) / 1000.0f;
		battery_status.voltage_cell_v[11] = static_cast<float>(tattu_message.cell_12_voltage) / 1000.0f;

		_battery_status_pub.publish(battery_status);
	}
}

int16_t TattuSocketCan::receive(CanFrame *received_frame)
{
	if ((_sock < 0) || (received_frame == nullptr)) {
		PX4_INFO("sock < 0");
		return -1;
	}

	int32_t result = recvmsg(_sock, &_recv_msg, MSG_DONTWAIT);
	if (result < 0) {
		PX4_INFO("recvmsg < 0");
		return result;
	}

	struct can_frame *recv_frame = (struct can_frame *)&_recv_frame;
	received_frame->extended_can_id = recv_frame->can_id & CAN_EFF_MASK;
	received_frame->payload_size = recv_frame->can_dlc;
	memcpy((void *)received_frame->payload, recv_frame->data, recv_frame->can_dlc);

	return recv_frame->can_dlc;
}

int TattuSocketCan::start()
{
	// There is a race condition at boot that sometimes causes opening of
	// socketcan0 to fail. We will delay 0.5s to be safe.
	uint32_t delay_us = 500000;
	ScheduleOnInterval(1000000 / SAMPLE_RATE, delay_us);
	return PX4_OK;
}

int TattuSocketCan::task_spawn(int argc, char *argv[])
{
	TattuSocketCan *instance = new TattuSocketCan();

	if (!instance) {
		PX4_ERR("driver allocation failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	instance->start();
	return 0;
}

int TattuSocketCan::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for reading data from the Tattu 12S 16000mAh smart battery.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("tattu_socketcan", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int TattuSocketCan::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	return print_usage("Unrecognized command.");
}

extern "C" __EXPORT int tattu_socketcan_main(int argc, char *argv[])
{
	return TattuSocketCan::main(argc, argv);
}
