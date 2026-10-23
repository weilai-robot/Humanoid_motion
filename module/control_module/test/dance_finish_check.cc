#include "control_module/dance_finish_core.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace xyber_x1_infer::rl_control_module;
namespace {
void Check(bool ok, const char* why) { if (!ok) throw std::runtime_error(why); }
struct Fixture {
  DanceFinishCore::Config cfg;
  DanceFinishCore::Input input;
  std::array<double, 12> unloaded{};
  Fixture() {
    cfg.lower.fill(-3); cfg.upper.fill(3); cfg.kp.fill(65); cfg.kd.fill(4);
    cfg.stand_kp.fill(300); cfg.stand_kd.fill(5);
    std::ifstream file(std::filesystem::path(__FILE__).parent_path()/"data/dance_x1_estimator_static.csv");
    std::string line; std::array<double, 59> row{}; bool found = false;
    while (std::getline(file, line)) {
      if (line.empty() || line[0] == '#') continue;
      std::replace(line.begin(), line.end(), ',', ' '); std::istringstream values(line);
      for (auto& v : row) Check(bool(values >> v), "invalid independent static fixture");
      found = true; break;
    }
    Check(found, "missing MuJoCo static fixture");
    std::copy_n(row.begin(), 4, input.orientation.begin());
    for (size_t j = 0; j < 12; ++j) {
      cfg.legs[j] = j; input.q[j] = cfg.nominal[j] = row[10+j];
      input.effort[j] = row[34+j]; unloaded[j] = row[46+j];
    }
  }
};
void Step(DanceFinishCore& core, Fixture& f, int start_ms, int end_ms, bool fresh = true) {
  for (int ms = start_ms; ms <= end_ms; ++ms) {
    if (fresh) f.input.source_ns = 1000000000LL+ms*1000000LL;
    core.Update(f.input, f.cfg.nominal, 1000000000LL+ms*1000000LL);
    Check(!core.Faulted(), "valid loaded fixture must not fault");
  }
}
void CheckNominalAndRecovery() {
  Fixture f; f.input.q[15] = 0.03;
  DanceFinishCore core(f.cfg); core.Begin(f.cfg.nominal, 1000000000LL);
  Step(core, f, 0, 2999);
  Check(core.GetPhase() != DanceFinishCore::Phase::kStandBlend && !core.StandingConfirmed(),
        "upper settling phase must finish before static capture");
  Step(core, f, 3000, 3999);
  Check(core.GetPhase() == DanceFinishCore::Phase::kStandBlend, "quiet loaded feet permit gradual standing handoff");
  Step(core, f, 4000, 6001);
  Check(core.StandingConfirmed(), "two seconds of quiet monitored standing are required");
  Check(std::abs(core.Targets()[15]-0.03) < 1e-12, "capture measured quiet stance, not a nominal pose jump");
  Check(core.Kp()[0] == 300 && core.Kd()[0] == 5, "stand gains blend to configured endpoint");
  f.input.contact_input_valid = false;
  bool saw_recovery = false;
  for (int ms = 6002; ms <= 6202; ++ms) {
    const auto old_phase = core.GetPhase(); const auto old_kp = core.Kp(); const auto old_target = core.Targets();
    Step(core, f, ms, ms);
    if (old_phase == DanceFinishCore::Phase::kStanding && core.GetPhase() == DanceFinishCore::Phase::kBrake) {
      Check(core.Kp() == old_kp && core.Targets() == old_target, "recovery must preserve targets and gains at its first tick");
      saw_recovery = true;
    }
  }
  Check(saw_recovery, "recovery boundary exercised");
  Check(!core.StandingConfirmed() && core.Recoveries() == 1 &&
        core.GetPhase() != DanceFinishCore::Phase::kStanding, "support loss resumes balance instead of keep");
  f.input.contact_input_valid = true;
  Step(core, f, 6203, 12500);
  Check(core.StandingConfirmed(), "fresh quiet support can recover and reverify");
}
void CheckMissingAndFrozenSupport() {
  Fixture f;
  std::copy(f.unloaded.begin(), f.unloaded.end(), f.input.effort.begin());
  DanceFinishCore hanging(f.cfg); hanging.Begin(f.cfg.nominal, 1000000000LL);
  Step(hanging, f, 0, 15000);
  Check(hanging.GetPhase() == DanceFinishCore::Phase::kLand && !hanging.StandingConfirmed(),
        "elapsed time alone must never force an unloaded robot into stand");
  Fixture frozen_fixture;
  DanceFinishCore frozen(frozen_fixture.cfg); frozen.Begin(frozen_fixture.cfg.nominal, 1000000000LL);
  frozen_fixture.input.source_ns = 1000000000LL;
  Step(frozen, frozen_fixture, 0, 10000, false);
  Check(!frozen.StandingConfirmed() && frozen.QuietSeconds() == 0, "replayed sensor stamps cannot accumulate a quiet hold");
  Fixture moving;
  moving.input.qd[0] = 1;
  DanceFinishCore moving_core(moving.cfg); moving_core.Begin(moving.cfg.nominal, 1000000000LL);
  Step(moving_core, moving, 0, 10000);
  Check(!moving_core.StandingConfirmed(), "moving joints must not pass standing gate");
}
void CheckLimitsAndFaults() {
  Fixture f; DanceFinishCore core(f.cfg);
  core.Begin(f.cfg.nominal, 1000000000LL);
  auto oversized = f.cfg.nominal; oversized.fill(100);
  auto previous = core.Targets();
  for (int ms = 0; ms <= 300; ++ms) {
    f.input.source_ns = 1000000000LL+ms*1000000LL;
    core.Update(f.input, oversized, f.input.source_ns);
    for (size_t j = 0; j < 29; ++j) {
      Check(core.Targets()[j] >= -3 && core.Targets()[j] <= 3, "all generated targets remain bounded");
      if (j >= 12) Check(std::abs(core.Targets()[j]-previous[j]) <= 0.00200000001, "upper targets obey 2 rad/s slew bound");
    }
    previous = core.Targets();
  }
  core.Update(f.input, oversized, 1500000000LL);
  Check(core.Faulted() && !core.StandingConfirmed(), "control gap over 80 ms must fault");
  DanceFinishCore tilted(f.cfg); tilted.Begin(f.cfg.nominal, 1000000000LL);
  const double angle = 40*std::acos(-1.0)/180;
  f.input.orientation = {std::cos(angle/2), 0, std::sin(angle/2), 0};
  tilted.Update(f.input, f.cfg.nominal, 1000000000LL);
  Check(tilted.Faulted(), "35 degree tilt protection remains active in all finishing phases");
  auto bad = f.cfg; bad.legs[1] = bad.legs[0];
  bool threw = false; try { DanceFinishCore invalid(bad); } catch (const std::exception&) { threw = true; }
  Check(threw, "ambiguous joint order rejected");
  bad = f.cfg; bad.contact_off = bad.contact_on;
  threw = false; try { DanceFinishCore invalid(bad); } catch (const std::exception&) { threw = true; }
  Check(threw, "invalid load hysteresis rejected");
}
}  // namespace

int main() {
  try {
    CheckNominalAndRecovery(); CheckMissingAndFrozenSupport(); CheckLimitsAndFaults();
    std::cout << "PASS: independent loaded/unloaded fixture, phase/quiet gates, measured capture, recovery, frozen stamps, bounds/slew, clock/tilt faults\n";
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
