#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <spot_micro_kinematics/utils.h>
#include "spot_micro_motion_cmd/command.h"

namespace smmc {

// Vorwärtsdeklaration
class Context;

class State {
public:
  virtual ~State() = default;
  virtual const char* name() const = 0;

  // Eintritt/Austritt
  virtual void onEnter(Context& ctx) {(void)ctx;}
  virtual void onExit (Context& ctx) {(void)ctx;}

  // Zyklische Aktualisierung; soll target_feet und Bodypose in ctx schreiben
  virtual void update(Context& ctx, const Command& cmd, float dt) = 0;
};

// ----------------- konkrete Zustände -----------------
class Idle    : public State { public: const char* name() const override { return "Idle"; }
  void onEnter(Context& ctx) override;
  void update(Context& ctx, const Command& cmd, float dt) override;
};

class Stand   : public State { public: const char* name() const override { return "Stand"; }
  void onEnter(Context& ctx) override;
  void update(Context& ctx, const Command& cmd, float dt) override;
};

class Walk    : public State { public: const char* name() const override { return "Walk"; }
  void onEnter(Context& ctx) override;
  void update(Context& ctx, const Command& cmd, float dt) override;
};

class TransitionIdle  : public State { public: const char* name() const override { return "TransitionIdle"; }
  void onEnter(Context& ctx) override;
  void update(Context& ctx, const Command& cmd, float dt) override;
};

class TransitionStand : public State { public: const char* name() const override { return "TransitionStand"; }
  void onEnter(Context& ctx) override;
  void update(Context& ctx, const Command& cmd, float dt) override;
};

// ----------------- FSM-Kontext -----------------
class Context {
public:
  rclcpp::Logger logger;

  // Kinematik-Zustand (Körperpose & Fußziele)
  smk::SpotMicroKinematics* kin{nullptr};
  smk::LegsFootPos          target_feet{};

  // Wechsel der Zustände
  void setState(std::unique_ptr<State> s){
    if(state){ state->onExit(*this); }
    state = std::move(s);
    if(state){ state->onEnter(*this); }
  }
  State* get(){ return state.get(); }

private:
  std::unique_ptr<State> state;
};

} // namespace smmc
