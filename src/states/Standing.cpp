/*
 * Copyright (c) 2018-2019, CNRS-UM LIRMM
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "Standing.h"

#include <mc_rtc/constants.h>

#include <lipm_walking/utils/clamp.h>

namespace lipm_walking
{

namespace
{
constexpr double COM_STIFFNESS = 5.; // standing has CoM set-point task
}

void states::Standing::configure(const mc_rtc::Configuration & config)
{
  config("autoplay", startWalking_);
  config("autoplay_plans", autoplay_plans_);
}

void states::Standing::start()
{
  auto & ctl = controller();
  auto & supportContact = ctl.supportContact();
  auto & targetContact = ctl.targetContact();

  planChanged_ = false;
  lastInterpolatorIter_ = ctl.planInterpolator.nbIter;
  leftFootRatio_ = ctl.leftFootRatio();
  startWalking_ = startWalking_ || ctl.config()("autoplay", false);
  ctl.isWalking = false;
  if(supportContact.surfaceName == "RightFootCenter")
  {
    leftFootContact_ = targetContact;
    rightFootContact_ = supportContact;
  }
  else if(supportContact.surfaceName == "LeftFootCenter")
  {
    leftFootContact_ = supportContact;
    rightFootContact_ = targetContact;
  }
  else
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("Unknown surface name: {}", supportContact.surfaceName);
  }

  if(ctl.isLastDSP())
  {
    ctl.loadFootstepPlan(ctl.plan.name);
  }

  ctl.setContacts({{ContactState::Left, leftFootContact_.pose}, {ContactState::Right, rightFootContact_.pose}});

  updatePlan(ctl.plan.name);
  updateTarget(leftFootRatio_);

  logger().addLogEntry("walking_phase", []() { return 3.; });
  ctl.stopLogSegment();

  if(startWalking_) // autoplay
  {
    auto plans = std::vector<std::string>{};
    if(autoplay_plans_.size())
    { // If defined, use plans from local state configuration
      plans = autoplay_plans_;
    }
    else
    { // otherwise use global plans
      plans = ctl.config()("autoplay_plans", std::vector<std::string>{});
    }
    if(plans.size() == 0)
    {
      startWalking_ = false;
      ctl.config().add("autoplay", false);
    }
    else
    {
      std::string plan = plans[0];
      plans.erase(plans.begin());
      ctl.config().add("autoplay_plans", plans);
      lastInterpolatorIter_++;
      updatePlan(plan);
    }
  }

  if(gui())
  {
    using namespace mc_rtc::gui;
    gui()->removeElement({"Walking", "Main"}, "Pause walking");
    gui()->addElement({"Walking", "Main"}, ComboInput("Footstep plan", ctl.planInterpolator.availablePlans(),
                                                      [&ctl]() { return ctl.plan.name; },
                                                      [this](const std::string & name) { updatePlan(name); }));
    gui()->addElement({"Walking", "Main"}, Button((supportContact.id == 0) ? "Start walking" : "Resume walking",
                                                  [this]() { startWalking(); }));
    gui()->addElement({"Standing"},
                      NumberInput("CoM target [0-1]", [this]() { return std::round(leftFootRatio_ * 10.) / 10.; },
                                  [this](double leftFootRatio) { updateTarget(leftFootRatio); }),
                      Label("Left foot force [N]",
                            [&ctl]() { return ctl.realRobot().forceSensor("LeftFootForceSensor").force().z(); }),
                      Label("Right foot force [N]",
                            [&ctl]() { return ctl.realRobot().forceSensor("RightFootForceSensor").force().z(); }),
                      Button("Go to left foot", [this]() { updateTarget(1.); }),
                      Button("Go to middle", [this]() { updateTarget(0.5); }),
                      Button("Go to right foot", [this]() { updateTarget(0.); }));
  }

  runState(); // don't wait till next cycle to update reference and tasks
}

void states::Standing::teardown()
{
  auto & ctl = controller();

  logger().removeLogEntry("walking_phase");

  if(gui())
  {
    gui()->removeCategory({"Standing"});
    gui()->removeElement({"Walking", "Main"}, "Footstep plan");
    gui()->removeElement({"Walking", "Main"}, "Gait");
    gui()->removeElement({"Walking", "Main"}, "Go to middle");
    gui()->removeElement({"Walking", "Main"}, "Resume walking");
    gui()->removeElement({"Walking", "Main"}, "Start walking");
  }
}

void states::Standing::runState()
{
  checkPlanUpdates();

  auto & ctl = controller();
  auto & pendulum = ctl.pendulum();

  Eigen::Vector3d comTarget = copTarget_ + Eigen::Vector3d{0., 0., ctl.plan.comHeight()};
  const Eigen::Vector3d & com_i = pendulum.com();
  const Eigen::Vector3d & comd_i = pendulum.comd();
  const Eigen::Vector3d & cop_f = copTarget_;

  double K = COM_STIFFNESS;
  double D = 2 * std::sqrt(K);
  Eigen::Vector3d comdd = K * (comTarget - com_i) - D * comd_i;
  Eigen::Vector3d n = ctl.supportContact().normal();
  double lambda = n.dot(comdd + mc_rtc::constants::gravity) / n.dot(com_i - cop_f);
  Eigen::Vector3d zmp = com_i - (mc_rtc::constants::gravity + comdd) / lambda;

  pendulum.integrateIPM(zmp, lambda, ctl.timeStep);
  ctl.leftFootRatio(leftFootRatio_);
  ctl.stabilizer()->target(pendulum.com(), pendulum.comd(), pendulum.comdd(), pendulum.zmp());
}

void states::Standing::checkPlanUpdates()
{
  auto & ctl = controller();

  if(ctl.plan.name == "external")
  {
    if(controller().datastore().has("Plugin::FSP::Plan"))
    {
      ctl.plan = controller().datastore().get<lipm_walking::FootstepPlan>("Plugin::FSP::Plan");
      const sva::PTransformd & X_0_lf = controller().robot().surfacePose("LeftFootCenter");
      const sva::PTransformd & X_0_rf = controller().robot().surfacePose("RightFootCenter");
      LOG_INFO("Current LeftFootCenter: " << X_0_lf.translation().transpose())
      LOG_INFO("Current RightFootCenter: " << X_0_rf.translation().transpose())
      ctl.plan.updateInitialTransform(X_0_lf, X_0_rf, 0);
      ctl.plan.rewind();
      controller().datastore().remove("Plugin::FSP::Plan");
      LOG_ERROR("Standing::Update::FootStepPlan")
    }
  }

  if(ctl.planInterpolator.nbIter > lastInterpolatorIter_)
  {
    ctl.loadFootstepPlan(ctl.planInterpolator.customPlanName());
    lastInterpolatorIter_ = ctl.planInterpolator.nbIter;
    planChanged_ = true;
  }
  if(planChanged_)
  {
    if(gui())
    {
      gui()->removeElement({"Walking", "Main"}, "Resume walking");
      gui()->removeElement({"Walking", "Main"}, "Start walking");
      gui()->addElement({"Walking", "Main"}, mc_rtc::gui::Button("Start walking", [this]() { startWalking(); }));
    }
    planChanged_ = false;
  }
}

void states::Standing::updateTarget(double leftFootRatio)
{
  auto & sole = controller().sole();
  if(!controller().stabilizer()->inDoubleSupport())
  {
    mc_rtc::log::error("Cannot update CoM target while in single support");
    return;
  }
  leftFootRatio = clamp(leftFootRatio, 0., 1., "Standing target");
  sva::PTransformd X_0_lfr =
      sva::interpolate(rightFootContact_.anklePose(sole), leftFootContact_.anklePose(sole), leftFootRatio);
  copTarget_ = X_0_lfr.translation();
  leftFootRatio_ = leftFootRatio;
}

bool states::Standing::checkTransitions()
{
  auto & ctl = controller();

  if(!startWalking_ || ctl.pauseWalking)
  {
    return false;
  }

  ctl.mpc().contacts(ctl.supportContact(), ctl.targetContact(), ctl.nextContact());
  ctl.mpc().phaseDurations(0., ctl.plan.initDSPDuration(), ctl.singleSupportDuration());
  if(ctl.updatePreview())
  {
    ctl.nextDoubleSupportDuration(ctl.plan.initDSPDuration());
    ctl.startLogSegment(ctl.plan.name);
    ctl.isWalking = true;
    output("DoubleSupport");
    return true;
  }
  return false;
}

void states::Standing::startWalking()
{
  auto & ctl = controller();
  if(ctl.isLastSSP())
  {
    mc_rtc::log::error("No footstep in contact plan");
    return;
  }
  startWalking_ = true;
  gui()->addElement({"Walking", "Main"},
                    mc_rtc::gui::Button("Pause walking", [&ctl]() { ctl.pauseWalkingCallback(/* verbose = */ true); }));
}

void states::Standing::updatePlan(const std::string & name)
{
  auto & ctl = controller();
  if(name.find("custom") != std::string::npos)
  {
    if(!ctl.planInterpolator.isShown)
    {
      ctl.planInterpolator.addGUIElements();
      ctl.planInterpolator.isShown = true;
    }
    if(name.find("backward") != std::string::npos)
    {
      ctl.planInterpolator.restoreBackwardTarget();
    }
    else if(name.find("forward") != std::string::npos)
    {
      ctl.planInterpolator.restoreForwardTarget();
    }
    else if(name.find("lateral") != std::string::npos)
    {
      ctl.planInterpolator.restoreLateralTarget();
    }
    ctl.loadFootstepPlan(ctl.planInterpolator.customPlanName());
  }
  else // new plan is not custom
  {
    if(ctl.planInterpolator.isShown)
    {
      ctl.planInterpolator.removeGUIElements();
      ctl.planInterpolator.isShown = false;
    }
    ctl.loadFootstepPlan(name);
  }
  planChanged_ = true;
}

} // namespace lipm_walking

EXPORT_SINGLE_STATE("Standing", lipm_walking::states::Standing)
