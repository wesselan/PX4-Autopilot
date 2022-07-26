/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file external_vision_control.cpp
 * Control functions for ekf external vision control
 */

#include "ekf.h"

void Ekf::controlExternalVisionFusion()
{
	// Check for new external vision data
	if (_ev_data_ready) {

		controlEvYawFusion();
		controlEvPosFusion();
		controlEvVelFusion();

		// record observation and estimate for use next time
		_ev_sample_delayed_prev = _ev_sample_delayed;

	} else if ((_control_status.flags.ev_pos || _control_status.flags.ev_vel ||  _control_status.flags.ev_yaw)
		   && isTimedOut(_time_last_ext_vision, (uint64_t)_params.reset_timeout_max)) {

		// Turn off EV fusion mode if no data has been received
		stopEvPosFusion();
		stopEvVelFusion();
		stopEvYawFusion();

		_warning_events.flags.vision_data_stopped = true;
		ECL_WARN("vision data stopped");
	}
}

void Ekf::controlEvPosFusion()
{
	if (_ev_data_ready) {

		// correct position and height for offset relative to IMU
		const Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
		const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;

		Vector3f pos{NAN, NAN, NAN};
		Matrix3f pos_cov{};

		switch (_ev_sample_delayed.pos_frame) {
		case PositionFrame::LOCAL_FRAME_NED: {
				pos = _ev_sample_delayed.pos - pos_offset_earth;
				pos_cov = matrix::diag(_ev_sample_delayed.posVar);
			}
			break;

		case PositionFrame::LOCAL_FRAME_FRD: {
				if (!_control_status.flags.ev_yaw) {
					// Calculate the quaternion delta that rotates from the EV to the EKF reference frame at the EKF fusion time horizon.
					const Quatf q_error((_state.quat_nominal * _ev_sample_delayed.quat.inversed()).normalized());
					const Dcmf R_ev_to_ekf = Dcmf(q_error);

					pos = R_ev_to_ekf * _ev_sample_delayed.pos - pos_offset_earth;
					pos_cov = R_ev_to_ekf * matrix::diag(_ev_sample_delayed.posVar) * R_ev_to_ekf.transpose();

				} else if (_control_status.flags.ev_yaw && !_control_status.flags.yaw_align) {
					// using EV frame
					pos = _ev_sample_delayed.pos - pos_offset_earth;
					pos_cov = matrix::diag(_ev_sample_delayed.posVar);
				}
			}
			break;
		}

		if (_control_status.flags.yaw_align && _control_status.flags.gps && _NED_origin_initialised) {
			// GPS active, adjust EV position to GNSS NED frame (if bias estimator valid)
			const matrix::Vector3f bias{_ev_gnss_frame_offset[0].getBias(), _ev_gnss_frame_offset[1].getBias(), _ev_gnss_frame_offset[2].getBias()};
			const matrix::Vector3f bias_var{_ev_gnss_frame_offset[0].getBiasVar(), _ev_gnss_frame_offset[1].getBiasVar(), _ev_gnss_frame_offset[2].getBiasVar()};

			if (bias_var.norm() < 0.01f) {
				pos -= bias;
			}
		}

		const Vector3f obs_var {
			fmaxf(pos_cov(0, 0), sq(0.01f)),
			fmaxf(pos_cov(1, 1), sq(0.01f)),
			fmaxf(pos_cov(2, 2), sq(0.01f))
		};

		_aid_src_ev_pos = createPositionAidSrcStatus(_ev_sample_delayed.time_us, pos, obs_var, fmaxf(_params.ev_pos_innov_gate, 1.f));

		// determine if we should use the horizontal position observations
		const bool quality_sufficient = ((_params.ev_quality_minimum <= 0) || ((_params.ev_quality_minimum > 0)
						 && (_ev_sample_delayed.quality >= _params.ev_quality_minimum)));

		const bool continuing_conditions_passing = isRecent(_time_last_ext_vision, 2 * EV_MAX_INTERVAL)
				&& _control_status.flags.tilt_align
				&& (_control_status.flags.ev_yaw || (_control_status.flags.yaw_align && (_ev_sample_delayed.pos_frame == PositionFrame::LOCAL_FRAME_NED)))
				&& (_params.fusion_mode & SensorFusionMask::USE_EXT_VIS_POS);

		const bool starting_conditions_passing = continuing_conditions_passing
				&& quality_sufficient;

		if (_control_status.flags.ev_pos) {
			if (continuing_conditions_passing) {

				if (_control_status_prev.flags.ev_pos && (_ev_sample_delayed.reset_counter != _ev_sample_delayed_prev.reset_counter)) {
					// reset count changed in EV sample, reset vision position unless GPS is active
					if (isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.ev_pos)) {
						_information_events.flags.reset_pos_to_vision = true;
						ECL_INFO("reset position to EV position");
						resetHorizontalPositionTo(Vector2f(pos));
						P.uncorrelateCovarianceSetVariance<2>(7, obs_var.xy());
					}

					if (_control_status.flags.ev_hgt) {
						_information_events.flags.reset_hgt_to_ev = true;
						ECL_INFO("reset position to EV");
						resetVerticalPositionTo(pos(2));
						P.uncorrelateCovarianceSetVariance<1>(9, obs_var(2));
					}

				} else {
					_aid_src_ev_pos.fusion_enabled[0] = PX4_ISFINITE(pos(0)) && quality_sufficient;
					_aid_src_ev_pos.fusion_enabled[1] = PX4_ISFINITE(pos(1)) && quality_sufficient;
					_aid_src_ev_pos.fusion_enabled[2] = PX4_ISFINITE(pos(2)) && quality_sufficient && _control_status.flags.ev_hgt;
					fusePosition(_aid_src_ev_pos);

					bool is_fusion_failing = isTimedOut(_aid_src_ev_pos.time_last_fuse[0], _params.reset_timeout_max)
								 || isTimedOut(_aid_src_ev_pos.time_last_fuse[1], _params.reset_timeout_max);

					if (is_fusion_failing) {
						if (_nb_ev_pos_reset_available > 0) {
							// Data seems good, attempt a reset
							_information_events.flags.reset_pos_to_vision = true;
							ECL_INFO("reset position to EV position");
							resetHorizontalPositionTo(Vector2f(pos));
							P.uncorrelateCovarianceSetVariance<2>(7, obs_var.xy());

							if (_control_status.flags.in_air) {
								_nb_ev_pos_reset_available--;
							}

						} else if (starting_conditions_passing) {
							// Data seems good, but previous reset did not fix the issue
							// something else must be wrong, declare the sensor faulty and stop the fusion
							//_control_status.flags.ev_pos_fault = true;
							stopEvPosFusion();

						} else {
							// A reset did not fix the issue but all the starting checks are not passing
							// This could be a temporary issue, stop the fusion without declaring the sensor faulty
							stopEvPosFusion();
						}
					}
				}

			} else {
				// Stop fusion but do not declare it faulty
				stopEvPosFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// Try to activate EV position fusion
				if (!_control_status.flags.ev_pos) {
					_control_status.flags.ev_pos = true;

					if (!_control_status.flags.gps) {
						_information_events.flags.reset_pos_to_vision = true;
						ECL_INFO("reset position to EV position");
						resetHorizontalPositionTo(Vector2f(pos));
						P.uncorrelateCovarianceSetVariance<2>(7, obs_var.xy());
					}

					_information_events.flags.starting_vision_pos_fusion = true;
					ECL_INFO("starting vision pos fusion");
				}

				if (_control_status.flags.ev_pos) {
					_nb_ev_pos_reset_available = 2;
				}
			}
		}

	} else if (_control_status.flags.ev_pos && isTimedOut(_time_last_ext_vision, _params.reset_timeout_max)) {
		// No data anymore. Stop until it comes back.
		stopEvPosFusion();
	}
}

void Ekf::controlEvVelFusion()
{
	if (_ev_data_ready) {

		// correct velocity for offset relative to IMU
		const Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
		const Vector3f vel_offset_body = _ang_rate_delayed_raw % pos_offset_body;
		const Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;

		// rotate measurement into correct earth frame if required
		Vector3f vel{NAN, NAN, NAN};
		Matrix3f vel_cov{};

		switch (_ev_sample_delayed.vel_frame) {
		case VelocityFrame::LOCAL_FRAME_NED: {
				vel = _ev_sample_delayed.vel - vel_offset_earth;
				vel_cov = _ev_sample_delayed.velCov;
			}
			break;

		case VelocityFrame::LOCAL_FRAME_FRD: {
				if (!_control_status.flags.ev_yaw) {
					// Calculate the quaternion delta that rotates from the EV to the EKF reference frame at the EKF fusion time horizon.
					const Quatf q_error((_state.quat_nominal * _ev_sample_delayed.quat.inversed()).normalized());
					const Dcmf R_ev_to_ekf = Dcmf(q_error);

					vel = R_ev_to_ekf * _ev_sample_delayed.vel - vel_offset_earth;
					vel_cov = R_ev_to_ekf * _ev_sample_delayed.velCov * R_ev_to_ekf.transpose();

				} else {
					// using EV frame
					vel = _ev_sample_delayed.vel;
					vel_cov = _ev_sample_delayed.velCov;
				}
			}
			break;

		case VelocityFrame::BODY_FRAME_FRD: {
				vel = _R_to_earth * (_ev_sample_delayed.vel - vel_offset_body);
				vel_cov = _R_to_earth * _ev_sample_delayed.velCov * _R_to_earth.transpose();
			}
			break;
		}

		_aid_src_ev_vel = createVelocityAidSrcStatus(_ev_sample_delayed.time_us, vel, vel_cov.diag(), fmaxf(_params.ev_vel_innov_gate, 1.f));

		const bool quality_sufficient = ((_params.ev_quality_minimum <= 0) || ((_params.ev_quality_minimum > 0)
						 && (_ev_sample_delayed.quality >= _params.ev_quality_minimum)));

		bool continuing_conditions_passing = isRecent(_time_last_ext_vision, 2 * EV_MAX_INTERVAL)
						     && _control_status.flags.tilt_align
						     && (_params.fusion_mode & SensorFusionMask::USE_EXT_VIS_VEL);

		if (_ev_sample_delayed.pos_frame == PositionFrame::LOCAL_FRAME_NED) {
			if (!_control_status.flags.yaw_align) {
				continuing_conditions_passing = false;
			}
		}

		const bool starting_conditions_passing = continuing_conditions_passing
				&& quality_sufficient;

		if (_control_status.flags.ev_vel) {

			if (continuing_conditions_passing) {

				if (_control_status_prev.flags.ev_vel && (_ev_sample_delayed.reset_counter != _ev_sample_delayed_prev.reset_counter)) {
					// reset count changed in EV sample
					if (isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.ev_vel)) {
						_information_events.flags.reset_vel_to_vision = true;
						ECL_INFO("reset to vision velocity");
						resetVelocityTo(vel);
						P.uncorrelateCovarianceSetVariance<3>(4, vel_cov.diag());
					}

				} else {
					_aid_src_ev_vel.fusion_enabled[0] = PX4_ISFINITE(vel(0)) && quality_sufficient;
					_aid_src_ev_vel.fusion_enabled[1] = PX4_ISFINITE(vel(1)) && quality_sufficient;
					_aid_src_ev_vel.fusion_enabled[2] = PX4_ISFINITE(vel(2)) && quality_sufficient;
					fuseVelocity(_aid_src_ev_vel);

					const bool is_fusion_failing = isTimedOut(_aid_src_ev_vel.time_last_fuse[0], _params.reset_timeout_max)
								       || isTimedOut(_aid_src_ev_vel.time_last_fuse[1], _params.reset_timeout_max)
								       || isTimedOut(_aid_src_ev_vel.time_last_fuse[2], _params.reset_timeout_max);

					if (is_fusion_failing) {
						if (_nb_ev_vel_reset_available > 0) {
							// Data seems good, attempt a reset
							_information_events.flags.reset_vel_to_vision = true;
							ECL_INFO("reset to vision velocity");
							resetVelocityTo(vel);
							P.uncorrelateCovarianceSetVariance<3>(4, vel_cov.diag());

							if (_control_status.flags.in_air) {
								_nb_ev_vel_reset_available--;
							}

						} else if (starting_conditions_passing) {
							// Data seems good, but previous reset did not fix the issue
							// something else must be wrong, declare the sensor faulty and stop the fusion
							//_control_status.flags.ev_vel_fault = true;
							stopEvVelFusion();

						} else {
							// A reset did not fix the issue but all the starting checks are not passing
							// This could be a temporary issue, stop the fusion without declaring the sensor faulty
							stopEvVelFusion();
						}
					}
				}

			} else {
				// Stop fusion but do not declare it faulty
				stopEvVelFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// Try to activate EV velocity fusion
				if (!_control_status.flags.ev_vel) {
					_control_status.flags.ev_vel = true;

					if (!_control_status.flags.gps) {
						// reset
						_information_events.flags.reset_vel_to_vision = true;
						ECL_INFO("reset to vision velocity");
						resetVelocityTo(vel);
						P.uncorrelateCovarianceSetVariance<3>(4, vel_cov.diag());
					}

					_information_events.flags.starting_vision_vel_fusion = true;
					ECL_INFO("starting vision vel fusion");
				}

				if (_control_status.flags.ev_vel) {
					_nb_ev_vel_reset_available = 2;
				}

			}
		}

	} else if (_control_status.flags.ev_vel && (isTimedOut(_time_last_ext_vision, _params.reset_timeout_max))) {
		// No data anymore. Stop until it comes back.
		stopEvVelFusion();
	}
}

void Ekf::controlEvYawFusion()
{
	if (_ev_data_ready) {
		// determine if we should use the yaw observation
		resetEstimatorAidStatusFlags(_aid_src_ev_yaw);
		const float measured_hdg = shouldUse321RotationSequence(_R_to_earth) ? getEuler321Yaw(
						   _ev_sample_delayed.quat) : getEuler312Yaw(_ev_sample_delayed.quat);
		const float ev_yaw_obs_var = fmaxf(_ev_sample_delayed.angVar, 1.e-4f);

		if (PX4_ISFINITE(measured_hdg)) {
			_aid_src_ev_yaw.timestamp_sample = _ev_sample_delayed.time_us;
			_aid_src_ev_yaw.observation = measured_hdg;
			_aid_src_ev_yaw.observation_variance = ev_yaw_obs_var;
			_aid_src_ev_yaw.fusion_enabled = _control_status.flags.ev_yaw;

			if (!_control_status.flags.ev_yaw) {
				// populate estimator_aid_src_ev_yaw with delta heading innovations for logging
				// use the change in yaw since the last measurement
				// const float measured_hdg_prev = shouldUse321RotationSequence(_R_to_earth) ? getEuler321Yaw(
				// 					_ev_sample_delayed_prev.quat) : getEuler312Yaw(_ev_sample_delayed_prev.quat);

				// // calculate the change in yaw since the last measurement
				// const float ev_delta_yaw = wrap_pi(measured_hdg - measured_hdg_prev);

				//_aid_src_ev_yaw.innovation = wrap_pi(getEulerYaw(_R_to_earth) - _yaw_pred_prev - ev_delta_yaw);
			}
		}

		const bool quality_sufficient = ((_params.ev_quality_minimum <= 0) || ((_params.ev_quality_minimum > 0)
						 && (_ev_sample_delayed.quality >= _params.ev_quality_minimum)));

		bool continuing_conditions_passing = isRecent(_time_last_ext_vision, 2 * EV_MAX_INTERVAL)
						     && (_params.fusion_mode & SensorFusionMask::USE_EXT_VIS_YAW)
						     && quality_sufficient;

		// && (!_control_status.flags.yaw_align || (_control_status.flags.yaw_align && _ev_sample_delayed.pos_frame == PositionFrame::LOCAL_FRAME_NED))
		// if (_control_status.flags.yaw_align && (_control_status.flags.gps || _control_status.flags.gps_yaw)) {
		// 	if (_ev_sample_delayed.pos_frame != PositionFrame::LOCAL_FRAME_NED) {
		// 		continuing_conditions_passing = false;
		// 	}
		// }

		bool starting_conditions_passing = continuing_conditions_passing && _control_status.flags.tilt_align;

		if (_control_status.flags.ev_yaw) {

			if (continuing_conditions_passing) {

				if (_control_status_prev.flags.ev_yaw && (_ev_sample_delayed.reset_counter != _ev_sample_delayed_prev.reset_counter)) {
					// reset count changed in EV sample
					const float yaw_new = getEulerYaw(_ev_sample_delayed.quat);
					const float yaw_new_variance = fmaxf(_ev_sample_delayed.angVar, sq(1.e-2f));
					resetQuatStateYaw(yaw_new, yaw_new_variance, true);

				} else {
					if (quality_sufficient) {
						const float innovation = wrap_pi(getEulerYaw(_R_to_earth) - measured_hdg);
						fuseYaw(innovation, ev_yaw_obs_var, _aid_src_ev_yaw);
					}

					const bool is_fusion_failing = isTimedOut(_aid_src_ev_yaw.time_last_fuse, _params.reset_timeout_max);

					if (is_fusion_failing) {
						if (_nb_ev_yaw_reset_available > 0) {
							// Data seems good, attempt a reset
							const float yaw_new = getEulerYaw(_ev_sample_delayed.quat);
							const float yaw_new_variance = fmaxf(_ev_sample_delayed.angVar, sq(1.e-2f));
							resetQuatStateYaw(yaw_new, yaw_new_variance, true);

							if (_control_status.flags.in_air) {
								_nb_ev_yaw_reset_available--;
							}

						} else if (starting_conditions_passing) {
							// Data seems good, but previous reset did not fix the issue
							// something else must be wrong, declare the sensor faulty and stop the fusion
							//_control_status.flags.ev_yaw_fault = true;
							stopEvYawFusion();

						} else {
							// A reset did not fix the issue but all the starting checks are not passing
							// This could be a temporary issue, stop the fusion without declaring the sensor faulty
							stopEvYawFusion();
						}
					}
				}

			} else {
				// Stop fusion but do not declare it faulty
				stopEvYawFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// Try to activate EV yaw fusion
				if (!_control_status.flags.ev_yaw) {

					if (_ev_sample_delayed.pos_frame == PositionFrame::LOCAL_FRAME_NED) {

						if (!_control_status.flags.yaw_align) {
							// reset yaw to EV and set yaw_align
							const float yaw_new = getEulerYaw(_ev_sample_delayed.quat);
							const float yaw_new_variance = fmaxf(_ev_sample_delayed.angVar, sq(1.e-2f));
							resetQuatStateYaw(yaw_new, yaw_new_variance, true);

							_control_status.flags.yaw_align = true;
						}

						_control_status.flags.ev_yaw = true;
						_information_events.flags.starting_vision_yaw_fusion = true;
						ECL_INFO("starting vision yaw fusion (aligned north)");

					} else if (_ev_sample_delayed.pos_frame == PositionFrame::LOCAL_FRAME_FRD) {
						if (!_control_status.flags.gps) {
							// turn on fusion of external vision yaw measurements and disable all magnetometer fusion
							stopMagFusion();
							stopGpsYawFusion();

							_control_status.flags.yaw_align = false;

							// reset yaw to EV
							ECL_INFO("reset yaw to EV");
							const float yaw_new = getEulerYaw(_ev_sample_delayed.quat);
							const float yaw_new_variance = fmaxf(_ev_sample_delayed.angVar, sq(1.e-2f));
							resetQuatStateYaw(yaw_new, yaw_new_variance, true);

							_control_status.flags.ev_yaw = true;
							_information_events.flags.starting_vision_yaw_fusion = true;
							ECL_INFO("starting vision yaw fusion");
						}
					}
				}

				if (_control_status.flags.ev_yaw) {
					_nb_ev_yaw_reset_available = 2;
				}
			}
		}

	} else if (_control_status.flags.ev_yaw && isTimedOut(_time_last_ext_vision, _params.reset_timeout_max)) {
		// No data anymore. Stop until it comes back.
		stopEvYawFusion();
	}
}

void Ekf::stopEvPosFusion()
{
	if (_control_status.flags.ev_pos) {
		ECL_INFO("stopping EV pos fusion");
		_control_status.flags.ev_pos = false;

		resetEstimatorAidStatus(_aid_src_ev_pos);
	}
}

void Ekf::stopEvVelFusion()
{
	if (_control_status.flags.ev_vel) {
		ECL_INFO("stopping EV vel fusion");
		_control_status.flags.ev_vel = false;

		resetEstimatorAidStatus(_aid_src_ev_vel);
	}
}

void Ekf::stopEvYawFusion()
{
	if (_control_status.flags.ev_yaw) {
		ECL_INFO("stopping EV yaw fusion");
		_control_status.flags.ev_yaw = false;

		if (_control_status.flags.yaw_align) {
			if (!_control_status.flags.mag_hdg && !_control_status.flags.mag_3D && !_control_status.flags.gps_yaw) {
				_control_status.flags.yaw_align = false;
			}
		}

		resetEstimatorAidStatus(_aid_src_ev_yaw);
	}
}

void Ekf::updateEvGnssFrameOffset()
{
	// TODO: EV resets


	// offset estimation using GNSS
	if (_ev_data_ready && (_delta_time_ev_us != 0)) {
		const float dt = math::constrain(1e-6f * _delta_time_ev_us, 0.f, 1.f);

		// EV GNSS offset
		for (int i = 0; i < 3; i++) {
			_ev_gnss_frame_offset[i].setMaxStateNoise(_params.gps_pos_noise);
			_ev_gnss_frame_offset[i].setProcessNoiseStdDev(0.01f); // process noise for offset estimation (m/s)
			_ev_gnss_frame_offset[i].predict(dt);
		}
	}

	if (_gps_data_ready && !_gps_intermittent
	    && _gps_checks_passed && _NED_origin_initialised) {

		// EV position
		Vector3f ev_position{_ev_sample_delayed.pos};
		Matrix3f ev_position_cov = matrix::diag(_ev_sample_delayed.posVar);

		switch (_ev_sample_delayed.pos_frame) {
		case PositionFrame::LOCAL_FRAME_NED: {
				ev_position = _ev_sample_delayed.pos;
				ev_position_cov = matrix::diag(_ev_sample_delayed.posVar);
			}
			break;

		case PositionFrame::LOCAL_FRAME_FRD: {
				// Calculate the quaternion delta that rotates from the EV to the EKF reference frame at the EKF fusion time horizon.
				const Quatf q_error((_state.quat_nominal * _ev_sample_delayed.quat.inversed()).normalized());
				const Dcmf R_ev_to_ekf = Dcmf(q_error);

				ev_position = R_ev_to_ekf * _ev_sample_delayed.pos;
				ev_position_cov = R_ev_to_ekf * matrix::diag(_ev_sample_delayed.posVar) * R_ev_to_ekf.transpose();
			}
			break;
		}

		// GNSS position
		const Vector3f gnss_position{
			_gps_sample_delayed.pos(0),
			_gps_sample_delayed.pos(1),
			-(_gps_sample_delayed.hgt - getEkfGlobalOriginAltitude())
		};

		const Vector3f offset = ev_position - gnss_position;

		_ev_gnss_frame_offset[0].fuseBias(offset(0), ev_position_cov(0, 0) + sq(_gps_sample_delayed.hacc));
		_ev_gnss_frame_offset[1].fuseBias(offset(1), ev_position_cov(1, 1) + sq(_gps_sample_delayed.hacc));
		_ev_gnss_frame_offset[2].fuseBias(offset(2), ev_position_cov(2, 2) + sq(_gps_sample_delayed.vacc));
	}
}
