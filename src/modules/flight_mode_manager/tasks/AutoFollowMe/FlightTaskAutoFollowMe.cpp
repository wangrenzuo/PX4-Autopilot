/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskAutoFollowMe.cpp
 */

#include "FlightTaskAutoFollowMe.hpp"

using namespace matrix;

constexpr float FlightTaskAutoFollowMe::_follow_position_matricies[4][9];

bool FlightTaskAutoFollowMe::init()
{
	_follow_offset = _param_nav_ft_dst.get() < 1.0F ? 1.0F : _param_nav_ft_dst.get();

	_responsiveness = math::constrain((float) _param_nav_ft_rs.get(), .1F, 1.0F);

	_follow_target_position = _param_nav_ft_fs.get();

	if ((_follow_target_position > FOLLOW_FROM_LEFT) || (_follow_target_position < FOLLOW_FROM_RIGHT)) {
		_follow_target_position = FOLLOW_FROM_BEHIND;
	}

	_rot_matrix = Dcmf(_follow_position_matricies[_follow_target_position]);

	return true;
}

bool FlightTaskAutoFollowMe::update()
{
	FlightTaskAuto::update();
	_position_setpoint = _target;
	matrix::Vector2f vel_sp = {};//_getTargetVelocityXY(); // TODO:
	_velocity_setpoint = matrix::Vector3f(vel_sp(0), vel_sp(1), 0.0f);


	vehicle_global_position_s vehicle_global_position{};
	_vehicle_global_position_sub.copy(&vehicle_global_position);




	struct map_projection_reference_s target_ref;
	follow_target_s target_motion_with_offset = {};
	uint64_t current_time = hrt_absolute_time();
	bool _radius_entered = false;
	bool _radius_exited = false;
	bool updated = false;
	float dt_ms = 0;

	if (_follow_target_sub.updated()) {
		follow_target_s target_motion;

		_target_updates++;

		// save last known motion topic
		_previous_target_motion = _current_target_motion;

		_follow_target_sub.copy(&target_motion);

		if (_current_target_motion.timestamp == 0) {
			_current_target_motion = target_motion;
		}

		_current_target_motion.timestamp = target_motion.timestamp;
		_current_target_motion.lat = (_current_target_motion.lat * (double)_responsiveness) + target_motion.lat * (double)(
						     1 - _responsiveness);
		_current_target_motion.lon = (_current_target_motion.lon * (double)_responsiveness) + target_motion.lon * (double)(
						     1 - _responsiveness);

	} else if (((current_time - _current_target_motion.timestamp) / 1000) > TARGET_TIMEOUT_MS && target_velocity_valid()) {
		reset_target_validity();
	}

	// update distance to target

	if (target_position_valid()) {

		// get distance to target

		map_projection_init(&target_ref, vehicle_global_position.lat, vehicle_global_position.lon);
		map_projection_project(&target_ref, _current_target_motion.lat, _current_target_motion.lon,
				       &_target_distance(0), &_target_distance(1));

	}

	// update target velocity
	if (target_velocity_valid() && updated) {

		dt_ms = ((_current_target_motion.timestamp - _previous_target_motion.timestamp) / 1000);

		// ignore a small dt
		if (dt_ms > 10.0F) {
			// get last gps known reference for target
			map_projection_init(&target_ref, _previous_target_motion.lat, _previous_target_motion.lon);

			// calculate distance the target has moved
			map_projection_project(&target_ref, _current_target_motion.lat, _current_target_motion.lon,
					       &(_target_position_delta(0)), &(_target_position_delta(1)));

			// update the average velocity of the target based on the position
			_est_target_vel = _target_position_delta / (dt_ms / 1000.0f);

			// if the target is moving add an offset and rotation
			if (_est_target_vel.length() > .5F) {
				_target_position_offset = _rot_matrix * _est_target_vel.normalized() * _follow_offset;
			}

			// are we within the target acceptance radius?
			// give a buffer to exit/enter the radius to give the velocity controller
			// a chance to catch up

			_radius_exited = ((_target_position_offset + _target_distance).length() > (float) TARGET_ACCEPTANCE_RADIUS_M * 1.5f);
			_radius_entered = ((_target_position_offset + _target_distance).length() < (float) TARGET_ACCEPTANCE_RADIUS_M);

			// to keep the velocity increase/decrease smooth
			// calculate how many velocity increments/decrements
			// it will take to reach the targets velocity
			// with the given amount of steps also add a feed forward input that adjusts the
			// velocity as the position gap increases since
			// just traveling at the exact velocity of the target will not
			// get any closer or farther from the target

			_step_vel = (_est_target_vel - _current_vel) + (_target_position_offset + _target_distance) * FF_K;
			_step_vel /= (dt_ms / 1000.0F * (float) INTERPOLATION_PNTS);
			_step_time_in_ms = (dt_ms / (float) INTERPOLATION_PNTS);

			// if we are less than 1 meter from the target don't worry about trying to yaw
			// lock the yaw until we are at a distance that makes sense

			if ((_target_distance).length() > 1.0F) {

				// yaw rate smoothing

				// this really needs to control the yaw rate directly in the attitude pid controller
				// but seems to work ok for now since the yaw rate cannot be controlled directly in auto mode

				_yaw_angle = get_bearing_to_next_waypoint(vehicle_global_position.lat, vehicle_global_position.lon,
						_current_target_motion.lat, _current_target_motion.lon);

				_yaw_rate = wrap_pi((_yaw_angle - _yaw) / (dt_ms / 1000.0f));

			} else {
				_yaw_angle = _yaw_rate = NAN;
			}
		}

//		warnx(" _step_vel x %3.6f y %3.6f cur vel %3.6f %3.6f tar vel %3.6f %3.6f dist = %3.6f (%3.6f) mode = %d yaw rate = %3.6f",
//				(double) _step_vel(0),
//				(double) _step_vel(1),
//				(double) _current_vel(0),
//				(double) _current_vel(1),
//				(double) _est_target_vel(0),
//				(double) _est_target_vel(1),
//				(double) (_target_distance).length(),
//				(double) (_target_position_offset + _target_distance).length(),
//				_follow_target_state,
//				(double) _yaw_rate);
	}

	if (target_position_valid()) {

		// get the target position using the calculated offset

		map_projection_init(&target_ref,  _current_target_motion.lat, _current_target_motion.lon);
		map_projection_reproject(&target_ref, _target_position_offset(0), _target_position_offset(1),
					 &target_motion_with_offset.lat, &target_motion_with_offset.lon);
	}

	// clamp yaw rate smoothing if we are with in
	// 3 degrees of facing target
	if (PX4_ISFINITE(_yaw_rate)) {
		if (fabsf(fabsf(_yaw_angle) - fabsf(_yaw)) < math::radians(3.f)) {
			_yaw_rate = NAN;
		}
	}

	// update state machine
	switch (_follow_target_state) {
	case TRACK_POSITION: {

			if (_radius_entered) {
				_follow_target_state = TRACK_VELOCITY;

			} else if (target_velocity_valid()) {
				set_follow_target_item(_param_nav_min_ft_ht.get(), target_motion_with_offset, _yaw_angle);
				// keep the current velocity updated with the target velocity for when it's needed
				_current_vel = _est_target_vel;

			} else {
				_follow_target_state = SET_WAIT_FOR_TARGET_POSITION;
			}

			break;
		}

	case TRACK_VELOCITY: {

			if (_radius_exited) {
				_follow_target_state = TRACK_POSITION;

			} else if (target_velocity_valid()) {

				if ((float)(current_time - _last_update_time) / 1000.0f >= _step_time_in_ms) {
					_current_vel += _step_vel;
					_last_update_time = current_time;
				}

				set_follow_target_item(_param_nav_min_ft_ht.get(), target_motion_with_offset, _yaw_angle);

			} else {
				_follow_target_state = SET_WAIT_FOR_TARGET_POSITION;
			}

			break;
		}

	case SET_WAIT_FOR_TARGET_POSITION: {
			// Climb to the minimum altitude
			// and wait until a position is received
			follow_target_s target{};

			// for now set the target at the minimum height above the uav
			target.lat = vehicle_global_position.lat;
			target.lon = vehicle_global_position.lon;
			target.alt = 0.0F;

			set_follow_target_item(_param_nav_min_ft_ht.get(), target, _yaw_angle);

			_follow_target_state = WAIT_FOR_TARGET_POSITION;
		}

	/* FALLTHROUGH */
	case WAIT_FOR_TARGET_POSITION: {
			//if (is_mission_item_reached() && target_velocity_valid()) {
			// TODO: decide if ITEM is reached
			if (target_velocity_valid()) {
				_target_position_offset(0) = _follow_offset;
				_follow_target_state = TRACK_POSITION;
			}

			break;
		}
	}


	return true;
}

void FlightTaskAutoFollowMe::reset_target_validity()
{
	_yaw_rate = NAN;
	_previous_target_motion = {};
	_current_target_motion = {};
	_target_updates = 0;
	_current_vel.zero();
	_step_vel.zero();
	_est_target_vel.zero();
	_target_distance.zero();
	_target_position_offset.zero();
	//reset_mission_item_reached();
	_follow_target_state = SET_WAIT_FOR_TARGET_POSITION;
}

bool FlightTaskAutoFollowMe::target_velocity_valid()
{
	// need at least 2 continuous data points for velocity estimate
	return (_target_updates >= 2);
}

bool FlightTaskAutoFollowMe::target_position_valid()
{
	// need at least 1 continuous data points for position estimate
	return (_target_updates >= 1);
}

void FlightTaskAutoFollowMe::set_follow_target_item(float min_clearance, follow_target_s &target, float yaw)
{
	// item->nav_cmd = NAV_CMD_DO_FOLLOW_REPOSITION;

	// /* use current target position */
	// item->lat = target.lat;
	// item->lon = target.lon;
	// item->altitude = _navigator->get_home_position()->alt;
	_yaw_setpoint = _yaw;

	//_position_setpoint = Vector3f(NAN, NAN, _position(2) - min_clearance;);
	Vector2f vel_sp_xy = Vector2f(_velocity).unit_or_zero() * _mc_cruise_speed;
	_velocity_setpoint = Vector3f(vel_sp_xy(0), vel_sp_xy(1), NAN);
}
