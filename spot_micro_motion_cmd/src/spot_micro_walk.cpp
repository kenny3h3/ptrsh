#include "spot_micro_walk.h"
#include <eigen3/Eigen/Geometry>
#include "spot_micro_transition_stand.h"
#include "spot_micro_motion_cmd.h"
#include "rate_limited_first_order_filter.h"

SpotMicroWalkState::SpotMicroWalkState() {
  ticks_ = 0;
  phase_index_ = 0;
  subphase_ticks_ = 0;
}

SpotMicroWalkState::~SpotMicroWalkState() {
  // Destructor
}

void SpotMicroWalkState::handleInputCommands(const smk::BodyState& body_state,
                                             const SpotMicroNodeConfig& smnc,
                                             const Command& cmd,
                                             SpotMicroMotionCmd* smmc,
                                             smk::BodyState* body_state_cmd_) {
  if (smnc.debug_mode) {
    std::cout << "In Spot Micro Walk State" << std::endl;
  }

  if (cmd.getStandCmd()) {
    changeState(smmc, std::make_unique<SpotMicroTransitionStandState>());
    return;
  }

  updatePhaseData();
  smk::LegsFootPos default_stance;
  default_stance.right_back = {0.0, smnc.default_stand_height, 0.0};
  default_stance.right_front = {smnc.stand_front_x_offset, smnc.default_stand_height, 0.0};
  default_stance.left_front = {smnc.stand_front_x_offset, smnc.default_stand_height, 0.0};
  default_stance.left_back = {smnc.stand_back_x_offset, smnc.default_stand_height, 0.0};
  cmd_state_ = body_state;
  cmd_state_.leg_feet_pos = stepGait(body_state, cmd, smnc, default_stance);
  cmd_state_.xyz_pos = stepBodyShift(body_state, cmd, smnc);
  *body_state_cmd_ = cmd_state_;
  smmc->publishServoProportionalCommand();
  ticks_++;
}

void SpotMicroWalkState::init(const smk::BodyState& body_state,
                              const SpotMicroNodeConfig& smnc,
                              const Command& cmd,
                              SpotMicroMotionCmd* smmc) {
  smnc_ = smnc;
  cmd_state_ = body_state;
  ticks_ = 0;
  phase_index_ = 0;
  subphase_ticks_ = 0;
  contact_feet_states_ = {false, false, false, false}; // Initialize all in stance
}

void SpotMicroWalkState::updatePhaseData() {
  phase_index_ = (ticks_ % smnc_.phase_length) / (smnc_.overlap_ticks + smnc_.swing_ticks);
  subphase_ticks_ = ticks_ % (smnc_.overlap_ticks + smnc_.swing_ticks);
  contact_feet_states_.right_back_in_swing = (smnc_.rb_contact_phases[phase_index_] == 0);
  contact_feet_states_.right_front_in_swing = (smnc_.rf_contact_phases[phase_index_] == 0);
  contact_feet_states_.left_front_in_swing = (smnc_.lf_contact_phases[phase_index_] == 0);
  contact_feet_states_.left_back_in_swing = (smnc_.lb_contact_phases[phase_index_] == 0);
}

smk::LegsFootPos SpotMicroWalkState::stepGait(const smk::BodyState& body_state,
                                              const Command& cmd,
                                              const SpotMicroNodeConfig& smnc,
                                              const smk::LegsFootPos& default_stance_feet_pos) {
  smk::LegsFootPos new_pos = body_state.leg_feet_pos;
  float swing_proportion = static_cast<float>(subphase_ticks_) / smnc.swing_ticks;

  if (contact_feet_states_.right_back_in_swing) {
    new_pos.right_back = swingLegController(body_state.leg_feet_pos.right_back, cmd, smnc, swing_proportion, default_stance_feet_pos.right_back);
  } else {
    new_pos.right_back = stanceController(body_state.leg_feet_pos.right_back, cmd, smnc);
  }
  // ... (analog für right_front, left_front, left_back)
  return new_pos;
}

smk::Point SpotMicroWalkState::stanceController(const smk::Point& foot_pos,
                                                const Command& cmd,
                                                const SpotMicroNodeConfig& smnc) {
  smk::Point new_pos = foot_pos;
  new_pos.x -= cmd.getXSpeedCmd() * smnc.dt;
  new_pos.z -= cmd.getYSpeedCmd() * smnc.dt;
  new_pos.y = smnc.default_stand_height; // Maintain height
  return new_pos;
}

smk::Point SpotMicroWalkState::swingLegController(const smk::Point& foot_pos,
                                                  const Command& cmd,
                                                  const SpotMicroNodeConfig& smnc,
                                                  float swing_proportion,
                                                  const smk::Point& default_stance_foot_pos) {
  smk::Point new_pos = foot_pos;
  float lift_height = smnc.z_clearance * sin(M_PI * swing_proportion);
  new_pos.y = default_stance_foot_pos.y + lift_height;
  new_pos.x = default_stance_foot_pos.x - cmd.getXSpeedCmd() * smnc.dt * swing_proportion;
  new_pos.z = default_stance_foot_pos.z - cmd.getYSpeedCmd() * smnc.dt * swing_proportion;
  return new_pos;
}

smk::Point SpotMicroWalkState::stepBodyShift(const smk::BodyState& body_state,
                                             const Command& cmd,
                                             const SpotMicroNodeConfig& smnc) {
  smk::Point shift = body_state.xyz_pos;
  if (smnc.body_shift_phases[phase_index_]) {
    shift.x += smnc.fwd_body_balance_shift * (contact_feet_states_.right_back_in_swing ? 1.0 : -1.0);
    shift.z += smnc.side_body_balance_shift * (contact_feet_states_.right_front_in_swing ? 1.0 : -1.0);
  }
  return shift;
}
