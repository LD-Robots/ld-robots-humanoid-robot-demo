#include "mc_com_posture_controller.h"

#include <mc_control/mc_controller.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/path.h>

#include <Eigen/Core>
#include <filesystem>

namespace mc_control
{

G1CoMPostureController::G1CoMPostureController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module,
                                               double dt,
                                               Backend backend)
: MCController(robot_module, dt, backend)
{
  qpsolver->addConstraintSet(contactConstraint);
  qpsolver->addConstraintSet(kinematicsConstraint);
  qpsolver->addConstraintSet(selfCollisionConstraint);
  qpsolver->addConstraintSet(*compoundJointConstraint);
  qpsolver->addTask(postureTask);

  comTask_ = std::make_shared<mc_tasks::CoMTask>(robots(), robots().robotIndex());
  torsoOriTask_ = std::make_shared<mc_tasks::OrientationTask>("torso_link", robots(), robots().robotIndex());
  qpsolver->addTask(torsoOriTask_);

  // Default task gains; overridden in reset once config is loaded
  comTask_->stiffness(10.0);
  comTask_->weight(200.0);
  postureTask->stiffness(5.0);
  postureTask->weight(10.0);

  mc_rtc::log::info("[G1CoMPosture] Posture task: stiffness={}, weight={}", 5.0, 10.0);

  stabilizerTask_ = std::make_shared<mc_tasks::lipm_stabilizer::StabilizerTask>(
      robots(), realRobots(), robots().robotIndex(), "LeftFoot", "RightFoot", "torso_link", dt);
  stabilizerTask_->name("g1_stabilizer");

  mc_rtc::log::success("G1CoMPostureController init done");
}

void G1CoMPostureController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  robot().forwardKinematics();
  robot().forwardVelocity();

  comTask_->reset();
  comTask_->com(robot().com());
  solver().addTask(comTask_);

  torsoOriTask_->reset();


  mc_rtc::Configuration cfg = config();
  const auto rootKeys = cfg.keys();
  mc_rtc::log::info("[G1CoMPosture] Config root keys count: {}", rootKeys.size());
  for(const auto & key : rootKeys)
  {
    mc_rtc::log::info("[G1CoMPosture] Config root key: {}", key);
  }
  mc_rtc::log::info("[G1CoMPosture] Controller name: {}", name_);
  if(cfg.has(name_))
  {
    cfg = cfg(name_);
  }
  else if(cfg.has("G1CoMPosture"))
  {
    cfg = cfg("G1CoMPosture");
  }
  if(!cfg.has("com") && !cfg.has("posture") && !cfg.has("stabilizer"))
  {
    const auto userControllers = mc_rtc::user_config_directory_path("controllers");
    std::filesystem::path cfgPath = std::filesystem::path(userControllers) / (name_ + ".yaml");
    if(!std::filesystem::exists(cfgPath))
    {
      cfgPath = std::filesystem::path(userControllers) / (name_ + ".conf");
    }
    if(std::filesystem::exists(cfgPath))
    {
      mc_rtc::log::info("[G1CoMPosture] Loading fallback config from {}", cfgPath.string());
      cfg.load(cfgPath.string());
    }
  }

  double comStiffness = 10.0;
  double comWeight = 200.0;
  if(cfg.has("com"))
  {
    auto comCfg = cfg("com");
    comCfg("stiffness", comStiffness);
    comCfg("weight", comWeight);
  }
  comTask_->stiffness(comStiffness);
  comTask_->weight(comWeight);

  double postureStiffness = 5.0;
  double postureWeight = 10.0;
  if(cfg.has("posture"))
  {
    auto postureCfg = cfg("posture");
    postureCfg("stiffness", postureStiffness);
    postureCfg("weight", postureWeight);
  }
  postureTask->stiffness(postureStiffness);
  postureTask->weight(postureWeight);

  if(cfg.has("posture") && cfg("posture").has("weights"))
  {
    auto jw = cfg("posture")("weights");
    std::map<std::string, double> weights;
    for(const auto & j : robot().refJointOrder())
    {
      if(jw.has(j))
      {
        weights[j] = jw(j);
      }
    }
    if(!weights.empty())
    {
      postureTask->jointWeights(weights);
    }
  }

  mc_rtc::log::info("[G1CoMPosture] Config com: stiffness={}, weight={}", comStiffness, comWeight);
  mc_rtc::log::info("[G1CoMPosture] Config posture: stiffness={}, weight={}", postureStiffness, postureWeight);

  double torsoStiffness = 2.0;
  double torsoWeight = 200.0;
  if(cfg.has("torsoOrientation"))
  {
    auto torsoCfg = cfg("torsoOrientation");
    torsoCfg("stiffness", torsoStiffness);
    torsoCfg("weight", torsoWeight);
  }
  torsoOriTask_->stiffness(torsoStiffness);
  torsoOriTask_->weight(torsoWeight);
  mc_rtc::log::info("[G1CoMPosture] Config torsoOrientation: stiffness={}, weight={}", torsoStiffness, torsoWeight);

  stabilizerTask_->reset();
  auto stabilizerConfig = stabilizerTask_->config();
  std::string leftSurface;
  std::string rightSurface;
  if(robot().hasSurface("LFullSole") && robot().hasSurface("RFullSole"))
  {
    leftSurface = "LFullSole";
    rightSurface = "RFullSole";
  }
  else if(robot().hasSurface("LeftFoot") && robot().hasSurface("RightFoot"))
  {
    leftSurface = "LeftFoot";
    rightSurface = "RightFoot";
  }

  bool enableStabilizer = true;
  if(config().has("stabilizer"))
  {
    mc_rbdyn::lipm_stabilizer::StabilizerConfiguration loadedConfig(config()("stabilizer"));
    stabilizerConfig = loadedConfig;
    enableStabilizer = config()("stabilizer")("enabled", true);
  }

  if(leftSurface.empty() || rightSurface.empty())
  {
    mc_rtc::log::warning("[G1CoMPosture] Missing foot surfaces, skipping stabilizer setup");
    enableStabilizer = false;
  }

  if(enableStabilizer)
  {
    if(stabilizerConfig.leftFootSurface.empty()) { stabilizerConfig.leftFootSurface = leftSurface; }
    if(stabilizerConfig.rightFootSurface.empty()) { stabilizerConfig.rightFootSurface = rightSurface; }
    stabilizerConfig.safetyThresholds.MIN_DS_PRESSURE = 0.0;
    stabilizerConfig.safetyThresholds.MIN_NET_TOTAL_FORCE_ZMP = 0.0;

    // Use config values if available, otherwise defaults
    if(config().has("stabilizer") && config()("stabilizer").has("contactStiffness"))
    {
      auto stiff = config()("stabilizer")("contactStiffness");
      stabilizerConfig.contactStiffness = sva::MotionVecd(
        {stiff[0], stiff[1], stiff[2]}, {stiff[3], stiff[4], stiff[5]});
    }
    else
    {
      stabilizerConfig.contactStiffness = sva::MotionVecd({4000, 4000, 4000}, {4000, 4000, 4000});
    }

    if(config().has("stabilizer") && config()("stabilizer").has("contactDamping"))
    {
      auto damp = config()("stabilizer")("contactDamping");
      stabilizerConfig.contactDamping = sva::MotionVecd(
        {damp[0], damp[1], damp[2]}, {damp[3], damp[4], damp[5]});
    }
    else
    {
      stabilizerConfig.contactDamping = sva::MotionVecd({400, 400, 400}, {400, 400, 400});
    }

    if(config().has("stabilizer") && config()("stabilizer").has("contactWeight"))
    {
      stabilizerConfig.contactWeight = config()("stabilizer")("contactWeight");
    }
    else
    {
      stabilizerConfig.contactWeight = 200000.0;
    }

    if(robot().hasBody("torso_link"))
    {
      stabilizerConfig.torsoBodyName = "torso_link";
    }
    else if(robot().hasBody("pelvis"))
    {
      stabilizerConfig.torsoBodyName = "pelvis";
    }

    stabilizerConfig.clampGains();
    stabilizerTask_->configure(stabilizerConfig);
    solver().addTask(stabilizerTask_);
    mc_rtc::log::success("[G1CoMPosture] Stabilizer enabled and configured");
  }
  else
  {
    mc_rtc::log::warning("[G1CoMPosture] Stabilizer disabled");
  }

  mc_rtc::log::info("[G1CoMPosture] CoM model: {} {} {}", robot().com().x(), robot().com().y(),
                    robot().com().z());
  mc_rtc::log::info("[G1CoMPosture] CoM target: {} {} {}", comTask_->com().x(), comTask_->com().y(),
                    comTask_->com().z());

  Eigen::Vector6d contactDof = Eigen::Vector6d::Ones();
  if(cfg.has("stabilizer"))
  {
    cfg("stabilizer")("contactDof", contactDof);
  }
  else
  {
    contactDof << 0, 0, 1, 1, 1, 0;
  }

  if(robot().hasSurface("LFullSole") && robot().hasSurface("RFullSole"))
  {
    addContact(Contact{env().name(), robot().name(), "AllGround", "LFullSole",
                       mc_rbdyn::Contact::defaultFriction, contactDof});
    addContact(Contact{env().name(), robot().name(), "AllGround", "RFullSole",
                       mc_rbdyn::Contact::defaultFriction, contactDof});
  }
  else if(robot().hasSurface("LeftFoot") && robot().hasSurface("RightFoot"))
  {
    addContact(Contact{env().name(), robot().name(), "AllGround", "LeftFoot",
                       mc_rbdyn::Contact::defaultFriction, contactDof});
    addContact(Contact{env().name(), robot().name(), "AllGround", "RightFoot",
                       mc_rbdyn::Contact::defaultFriction, contactDof});
  }
  else
  {
    mc_rtc::log::warning("No foot surfaces found for {}, skipping contact setup", robot().name());
  }
}

} // namespace mc_control

MULTI_CONTROLLERS_CONSTRUCTOR("G1CoMPosture",
                              mc_control::G1CoMPostureController(rm, dt, mc_control::MCController::Backend::Tasks),
                              "G1CoMPosture_TVM",
                              mc_control::G1CoMPostureController(rm, dt, mc_control::MCController::Backend::TVM))
