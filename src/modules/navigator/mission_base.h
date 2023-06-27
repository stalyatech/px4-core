/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file mission_base.h
 *
 * Mission base mode class that can be used for modes interacting with a mission.
 *
 */

#pragma once

#include <cstdint>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>
#include <dataman_client/DatamanClient.hpp>
#include <uORB/topics/mission.h>
#include <uORB/topics/navigator_mission_item.h>
#include <uORB/topics/position_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

#include "mission_block.h"
#include "navigation.h"

using namespace time_literals;

class Navigator;

class MissionBase : public MissionBlock, public ModuleParams
{
public:
	MissionBase(Navigator *navigator, int32_t dataman_cache_size_signed);
	~MissionBase() override = default;

	virtual void on_inactive() override;
	virtual void on_inactivation() override;
	virtual void on_activation() override;
	virtual void on_active() override;

protected:
	// Work Item corresponds to the sub-mode set on the "MAV_CMD_DO_SET_MODE" MAVLink message
	enum class WorkItemType {
		WORK_ITEM_TYPE_DEFAULT,		/**< default mission item */
		WORK_ITEM_TYPE_TAKEOFF,		/**< takeoff before moving to waypoint */
		WORK_ITEM_TYPE_MOVE_TO_LAND,	/**< move to land waypoint before descent */
		WORK_ITEM_TYPE_ALIGN,		/**< align for next waypoint */
		WORK_ITEM_TYPE_TRANSITION_AFTER_TAKEOFF,
		WORK_ITEM_TYPE_MOVE_TO_LAND_AFTER_TRANSITION,
		WORK_ITEM_TYPE_PRECISION_LAND
	} _work_item_type{WorkItemType::WORK_ITEM_TYPE_DEFAULT};	/**< current type of work to do (sub mission item) */

	enum class MissionType {
		MISSION_TYPE_NONE,
		MISSION_TYPE_MISSION
	} _mission_type{MissionType::MISSION_TYPE_NONE};

	void getPreviousPositionItems(int32_t start_index, int32_t items_index[], size_t &num_found_items,
				      uint8_t max_num_items);
	void getNextPositionItems(int32_t start_index, int32_t items_index[], size_t &num_found_items,
				  uint8_t max_num_items);
	bool hasMissionLandStart() const { return _mission.land_start_index > 0;};
	int goToNextItem(bool execute_jump);
	int goToPreviousItem(bool execute_jump);
	int goToItem(int32_t index, bool execute_jump, bool mission_direction_backward = false);
	int goToPreviousPositionItem(bool execute_jump);
	int goToNextPositionItem(bool execute_jump);
	int goToMissionLandStart();
	int setMissionToClosestItem(double lat, double lon, float alt, float home_alt, const vehicle_status_s &vehicle_status);

	int initMission();
	void resetMission();
	void resetMissionJumpCounter();

	int getNonJumpItem(int32_t &mission_index, mission_item_s &mission, bool execute_jump, bool write_jumps,
			   bool mission_direction_backward = false);
	void setMissionIndex(int32_t index);
	int writeMission(mission_s &mission);
	int readMission(mission_s &read_mission) const;
	int readMissionItem(mission_item_s &read_mission_item, size_t index) const;
	int writeMissionItem(const mission_item_s &mission_item, size_t index) const ;
	bool isMissionValid(mission_s &mission) const;
	void findLandStartItem();
	static constexpr uint16_t MAX_JUMP_ITERATION{10u};
	static constexpr hrt_abstime MAX_DATAMAN_LOAD_WAIT{500_ms};


	/**
	 * On mission update
	 * Change behaviour after external mission update.
	 * @param[in] has_mission_items_changed flag if the mission items have been changed.
	 */
	virtual void onMissionUpdate(bool has_mission_items_changed);

	/**
	 * Update mission topic
	 */
	virtual void update_mission();

	/**
	 * Move on to next mission item or switch to loiter
	 */
	void advance_mission();

	/**
	 * @brief Configures mission items in current setting
	 *
	 * Configure the mission items depending on current mission item index and settings such
	 * as terrain following, etc.
	 */
	void set_mission_items();

	/**
	 * Set the mission result
	 */
	void set_mission_result();

	/**
	 * @brief Reset the item cache
	 */
	void resetItemCache();

	virtual void setActiveMissionItems() = 0;
	virtual bool setNextMissionItem() = 0;
	void setEndOfMissionItems();
	void publish_navigator_mission_item();
	bool position_setpoint_equal(const position_setpoint_s *p1, const position_setpoint_s *p2) const;

	bool _is_current_planned_mission_item_valid{false};
	bool _mission_has_been_activated{false};
	bool _initialized_mission_checked{false};
	bool _need_takeoff{true};					/**< if true, then takeoff must be performed before going to the first waypoint (if needed) */
	bool _system_disarmed_while_inactive{false};
	mission_s _mission;

	DatamanCache _dataman_cache{"mission_dm_cache_miss", 10};
	DatamanClient	&_dataman_client = _dataman_cache.client();

	uORB::Subscription _mission_sub{ORB_ID(mission)};
	uORB::SubscriptionData<vehicle_land_detected_s> _land_detected_sub{ORB_ID(vehicle_land_detected)};	/**< vehicle land detected subscription */
	uORB::SubscriptionData<vehicle_status_s> _vehicle_status_sub{ORB_ID(vehicle_status)};	/**< vehicle status subscription */
	uORB::SubscriptionData<vehicle_global_position_s> _global_pos_sub{ORB_ID(vehicle_global_position)};	/**< global position subscription */
	uORB::Publication<navigator_mission_item_s> _navigator_mission_item_pub{ORB_ID::navigator_mission_item};
	uORB::Publication<mission_s> _mission_pub{ORB_ID(mission)};
private:

	void updateDatamanCache();
	void updateMavlinkMission();

	/**
	 * Check whether a mission is ready to go
	 */
	void check_mission_valid();

	/**
	 * Reset mission
	 */
	void checkMissionRestart();

	/**
	 * Set a mission item as reached
	 */
	void set_mission_item_reached();

	/**
	 * Updates the heading of the vehicle. Rotary wings only.
	 */
	void heading_sp_update();

	/**
	 * Update the cruising speed setpoint.
	 */
	void cruising_speed_sp_update();

	/**
	 * Abort landing
	 */
	void do_abort_landing();

	/**
	 * Inform about a changed mission item after a DO_JUMP
	 */
	void report_do_jump_mission_changed(int index, int do_jumps_remaining);

	/**
	 * @brief Cache the mission items containing gimbal, camera mode and trigger commands
	 *
	 * @param mission_item The mission item to cache if applicable
	 */
	void cacheItem(const mission_item_s &mission_item);

	/**
	 * @brief Update the cached items up to the given index
	 *
	 * @param end_index The index to update up to
	 */
	void updateCachedItemsUpToIndex(int end_index);

	/**
	 * @brief Replay the cached gimbal and camera mode items
	 */
	void replayCachedGimbalCameraItems();

	/**
	 * @brief Replay the cached trigger items
	 *
	 */
	void replayCachedTriggerItems();

	/**
	 * @brief Check if there are cached gimbal or camera mode items to be replayed
	 *
	 * @return true if there are cached items
	 */
	bool haveCachedGimbalOrCameraItems();

	/**
	 * @brief Check if the camera was triggering
	 *
	 * @return true if there was a camera trigger command in the cached items that didn't disable triggering
	 */
	bool cameraWasTriggering();

	int32_t _load_mission_index{-1};
	int32_t _dataman_cache_size_signed;

	int _inactivation_index{-1}; // index of mission item at which the mission was paused. Used to resume survey missions at previous waypoint to not lose images.
	bool _align_heading_necessary{false}; // if true, heading of vehicle needs to be aligned with heading of next waypoint. Used to create new mission items for heading alignment.

	mission_item_s _last_gimbal_configure_item {};
	mission_item_s _last_gimbal_control_item {};
	mission_item_s _last_camera_mode_item {};
	mission_item_s _last_camera_trigger_item {};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MIS_DIST_1WP>) _param_mis_dist_1wp,
		(ParamInt<px4::params::MIS_MNT_YAW_CTL>) _param_mis_mnt_yaw_ctl
	)
};
