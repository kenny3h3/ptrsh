#include "spot_micro_stand.h"
#include "spot_micro_transition_idle.h"
#include "spot_micro_motion_cmd.h"
#include "spot_micro_kinematics/spot_micro_kinematics.h"

SpotMicroStandState::SpotMicroStandState() {
  // Constructor
}

SpotMicroStandState::~SpotMicroStandState() {
  // Destructor
}

void SpotMicroStandState::handleInputCommands(const smk::BodyState& body_state,
                                              const SpotMicroNodeConfig& smnc,
                                              const Command& cmd,
                                              SpotMicroMotionCmd* smmc,
                                              smk::BodyState* body_state_cmd_) {
  if (smnc.debug_mode) {
    std::cout << "In Spot Micro Stand State" << std::endl;
  }

  // Check if idle command issued, if so, transition to idle state
  if (cmd.getIdleCmd() == true) {
    changeState(smmc, std::make_unique<SpotMicroTransitionIdleState>());

  } else {
    // Command stand position
    smk::BodyState stand_state = body_state;
    stand_state.xyz_pos.y = smnc.default_stand_height;
    stand_state.leg_feet_pos.right_front.x += smnc.stand_front_x_offset;
    stand_state.leg_feet_pos.right_back.x += smnc.stand_back_x_offset;
    stand_state.leg_feet_pos.left_front.x += smnc.stand_front_x_offset;
    stand_state.leg_feet_pos.left_back.x += smnc.stand_back_x_offset;
    *body_state_cmd_ = stand_state;
    smmc->publishServoProportionalCommand();
  }
}

void SpotMicroStandState::init(const smk::BodyState& body_state,
                               const SpotMicroNodeConfig& smnc,
                               const Command& cmd,
                               SpotMicroMotionCmd* smmc) {
  cmd_state_ = body_state;
  initBodyStateFilters(smnc.dt, smnc.transit_tau, smnc.transit_rl, smnc.transit_angle_rl, body_state, &angle_cmd_filters_);
}
