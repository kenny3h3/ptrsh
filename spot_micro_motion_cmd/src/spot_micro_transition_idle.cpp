#include "spot_micro_transition_idle.h"
#include "spot_micro_idle.h"
#include "spot_micro_motion_cmd.h"
#include "spot_micro_kinematics/spot_micro_kinematics.h"

SpotMicroTransitionIdleState::SpotMicroTransitionIdleState() {
  // Constructor
}

SpotMicroTransitionIdleState::~SpotMicroTransitionIdleState() {
  // Destructor
}

void SpotMicroTransitionIdleState::handleInputCommands(const smk::BodyState& body_state,
                                                       const SpotMicroNodeConfig& smnc,
                                                       const Command& cmd,
                                                       SpotMicroMotionCmd* smmc,
                                                       smk::BodyState* body_state_cmd_) {
  if (smnc.debug_mode) {
    std::cout << "In Spot Micro Transition Idle State" << std::endl;
  }

  // Update filters
  runFilters(&body_state_filters_);
  assignFilterValuesToBodyState(body_state_filters_, body_state_cmd_);

  // Check if transition complete
  if (checkBodyStateEquality(*body_state_cmd_, end_body_state_, 0.01f)) {
    changeState(smmc, std::make_unique<SpotMicroIdleState>());
  }
}

void SpotMicroTransitionIdleState::init(const smk::BodyState& body_state,
                                        const SpotMicroNodeConfig& smnc,
                                        const Command& cmd,
                                        SpotMicroMotionCmd* smmc) {
  start_body_state_ = body_state;
  end_body_state_ = body_state; // Set to idle position (e.g., legs at rest)
  end_body_state_.xyz_pos.y = smnc.lie_down_height;
  end_body_state_.leg_feet_pos.right_back.x += smnc.lie_down_feet_x_offset;
  end_body_state_.leg_feet_pos.right_front.x += smnc.lie_down_feet_x_offset;
  end_body_state_.leg_feet_pos.left_front.x += smnc.lie_down_feet_x_offset;
  end_body_state_.leg_feet_pos.left_back.x += smnc.lie_down_feet_x_offset;
  initBodyStateFilters(smnc.dt, smnc.transit_tau, smnc.transit_rl, smnc.transit_angle_rl, body_state, &body_state_filters_);
  setBodyStateFilterCommands(end_body_state_, &body_state_filters_);
}
