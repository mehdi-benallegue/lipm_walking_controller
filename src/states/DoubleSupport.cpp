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

#include "DoubleSupport.h"

namespace lipm_walking
{

using ContactState = mc_tasks::lipm_stabilizer::ContactState;

void states::DoubleSupport::start()
{
  auto & ctl = controller();

  double phaseDuration = ctl.doubleSupportDuration(); // careful! side effect here

  duration_ = phaseDuration;
  initLeftFootRatio_ = ctl.leftFootRatio();
  remTime_ = (phaseDuration > ctl.timeStep) ? phaseDuration : -ctl.timeStep;
  stateTime_ = 0.;
  stopDuringThisDSP_ = ctl.pauseWalking;
  if(phaseDuration > ctl.timeStep)
  {
    timeSinceLastPreviewUpdate_ = 2 * PREVIEW_UPDATE_PERIOD; // update at transition...
  }
  else // ... unless DSP duration is zero
  {
    timeSinceLastPreviewUpdate_ = 0.;
  }

  if(ctl.plan.name == "external")
  {
    if(controller().datastore().has("Plugin::FSP::Plan"))
    {
      //Recive the external footstep plan via mc_datastore.
      ctl.plan = controller().datastore().get<lipm_walking::FootstepPlan>("Plugin::FSP::Plan");
      const sva::PTransformd & X_0_lf = controller().robot().surfacePose("LeftFootCenter");
      const sva::PTransformd & X_0_rf = controller().robot().surfacePose("RightFootCenter");
      ctl.plan.updateInitialTransform(X_0_lf, X_0_rf, 0);
      ctl.plan.rewind();
      controller().datastore().remove("Plugin::FSP::Plan");
      mc_rtc::log::info("Current LeftFootCenter: {}", X_0_lf.translation().transpose());
      mc_rtc::log::info("Current RightFootCenter: {}", X_0_rf.translation().transpose());
      mc_rtc::log::info("DoubleSupport::Update::FootStepPlan");
    }
    /*
    if(ros::param::has("online_footstep_plan"))
    {
      ros::param::get("online_footstep_plan", ctl.planInterpolator.online);
    }
    ROS_ERROR("get param");
    */
    if(ctl.planInterpolator.online)
    {
      controller().datastore().remove("Plugin::FSP::Request");
      controller().datastore().make<bool>("Plugin::FSP::Request", true);
    }
    else
    {
      controller().datastore().remove("Plugin::FSP::Request");
      controller().datastore().make<bool>("Plugin::FSP::Request", false);
    }
  }

  const std::string & targetSurfaceName = ctl.targetContact().surfaceName;
  auto actualTargetPose = ctl.controlRobot().surfacePose(targetSurfaceName);
  ctl.plan.goToNextFootstep(actualTargetPose);

  if(ctl.isLastDSP()) // called after goToNextFootstep
  {
    stopDuringThisDSP_ = true; //TODO: We have to comment out this line to walk permanently. Or set some condition to stop/restart walking.
  }

  if(ctl.prevContact().surfaceName == "LeftFootCenter")
  {
    ctl.setContacts({{ContactState::Left, ctl.prevContact().pose}, {ContactState::Right, ctl.supportContact().pose}});
    targetLeftFootRatio_ = 0.;
  }
  else // (ctl.prevContact().surfaceName == "RightFootCenter")
  {
    ctl.setContacts({{ContactState::Left, ctl.supportContact().pose}, {ContactState::Right, ctl.prevContact().pose}});
    targetLeftFootRatio_ = 1.;
  }
  if(stopDuringThisDSP_)
  {
    targetLeftFootRatio_ = 0.5;
  }

  logger().addLogEntry("rem_phase_time", [this]() { return remTime_; });
  logger().addLogEntry("walking_phase", []() { return 2.; });

  if(stopDuringThisDSP_)
  {
    ctl.pauseWalking = false;
  }

  runState(); // don't wait till next cycle to update reference and tasks
}

void states::DoubleSupport::teardown()
{
  logger().removeLogEntry("rem_phase_time");
  logger().removeLogEntry("walking_phase");
}

void states::DoubleSupport::runState()
{
  auto & ctl = controller();
  double dt = ctl.timeStep;

  if(remTime_ > 0 && timeSinceLastPreviewUpdate_ > PREVIEW_UPDATE_PERIOD
     && !(stopDuringThisDSP_ && remTime_ < PREVIEW_UPDATE_PERIOD))
  {
    updatePreview();
  }

  double x = clamp(remTime_ / duration_, 0., 1.);
  ctl.leftFootRatio(x * initLeftFootRatio_ + (1. - x) * targetLeftFootRatio_);

  ctl.preview->integrate(pendulum(), dt);
  pendulum().completeIPM(ctl.prevContact().p(), ctl.prevContact().normal());
  pendulum().resetCoMHeight(ctl.plan.comHeight(), ctl.prevContact().p(), ctl.prevContact().normal());
  controller().stabilizer()->target(pendulum().com(), pendulum().comd(), pendulum().comdd(), pendulum().zmp());

  remTime_ -= dt;
  stateTime_ += dt;
  timeSinceLastPreviewUpdate_ += dt;
}

bool states::DoubleSupport::checkTransitions()
{
  auto & ctl = controller();
  if(!stopDuringThisDSP_ && remTime_ < 0.)
  {
    output("SingleSupport");
    return true;
  }
  if(stopDuringThisDSP_ && remTime_ < -0.5)
  {
    if(!ctl.isLastDSP())
    {
      ctl.plan.restorePreviousFootstep(); // current one is for next SSP
    }
    output("Standing");
    return true;
  }
  return false;
}

void states::DoubleSupport::updatePreview()
{
  auto & ctl = controller();
  ctl.mpc().contacts(ctl.prevContact(), ctl.supportContact(), ctl.targetContact());
  if(stopDuringThisDSP_)
  {
    ctl.mpc().phaseDurations(0., remTime_, 0.);
  }
  else
  {
    ctl.mpc().phaseDurations(0., remTime_, ctl.singleSupportDuration());
  }
  if(ctl.updatePreview())
  {
    timeSinceLastPreviewUpdate_ = 0.;
  }
  else
  {
    mc_rtc::log::warning("No capture trajectory, resuming walking");
    stopDuringThisDSP_ = false;
  }
}

} // namespace lipm_walking

EXPORT_SINGLE_STATE("DoubleSupport", lipm_walking::states::DoubleSupport)
