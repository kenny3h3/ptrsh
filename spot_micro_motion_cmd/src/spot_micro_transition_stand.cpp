#include "spot_micro_transition_stand.h"
#include "spot_micro_stand.h"
#include "spot_micro_motion_cmd.h"
#include "spot_micro_kinematics/spot_micro_kinematics.h"

SpotMicroTransitionStandState::SpotMicroTransitionStandState() {
  // Constructor
}

SpotMicroTransitionStandState::~SpotMicroTransitionStandState() {
  // Destructor
}

void SpotMicroTransitionStandState::handleInputCommands(const smk::BodyState& body_state,
                                                        const SpotMicroNodeConfig& smnc,
                                                        const Command& cmd,
                                                        SpotMicroMotionCmd* smmc,
                                                        smk::BodyState* body_state_cmd_) {
  if (smnc.debug_mode) {
    std::cout << "In Spot Micro Transition Stand State" << std::endl;
  }

  // Update filters
  runFilters(&body_state_filters_);
  assignFilterValuesToBodyState(body_state_filters_, body_state_cmd_);

  // Check if transition complete
  if (checkBodyStateEquality(*body_state_cmd_, end_body_state_, 0.01f)) {
    changeState(smmc, std::make_unique<SpotMicroStandState>());
  }
}

void SpotMicroTransitionStandState::init(const smk::BodyState& body_state,
                                         const SpotMicroNodeConfig& smnc,
                                         const Command& cmd,
                                         SpotMicroMotionCmd* smmc) {
  start_body_state_ = body_state;
  end_body_state_ = body_state; // Set to stand position
  end_body_state_.xyz_pos.y = smnc.default_stand_height;
  end_body_state_.leg_feet_pos.right_front.x += smnc.stand_front_x_offset;
  end_body_state_.leg_feet_pos.right_back.x += smnc.stand_back_x_offset;
  end_body_state_.leg_feet_pos.left_front.x += smnc.stand_front_x_offset;
  end_body_state_.leg_feet_pos.left_back.x += smnc.stand_back_x_offset;
  initBodyStateFilters(smnc.dt, smnc.transit_tau, smnc.transit_rl, smnc.transit_angle_rl, body_state, &body_state_filters_);
  setBodyStateFilterCommands(end_body_state_, &body_state_filters_);
}
