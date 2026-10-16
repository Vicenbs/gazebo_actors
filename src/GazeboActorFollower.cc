/*
 * GazeboActorFollower.cc
 *
 * Generic Gazebo Sim Harmonic System Plugin to tightly couple an animated <actor>
 * with a physical kinematic collision model (<model>).
 *
 * Compatible with any Gazebo world (flat terrain, slopes, indoor, outdoor).
 *
 * Features:
 * - Direct TrajectoryPose & Animation State propagation to Gazebo ECM:
 *   Marks TrajectoryPose, AnimationName, and AnimationTime with
 *   ComponentState::OneTimeChange so SceneBroadcaster sends live state to GUI.
 * - Zero-base Origin Lock: Keeps base components::Pose zeroed so that
 *   TrajectoryPose directly represents global world coordinates.
 * - Dynamic Animation Blending: Smoothly switches to 'walk' when moving,
 *   and seamlessly transitions to 'stand' idle pose when stopped.
 * - Rate Smoothing: Bridges discrete ROS collider updates (e.g. 20 Hz) with
 *   Gazebo Sim's 250 Hz physics/render loop for fluid motion in any world.
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

namespace gazebo_actors {

class ActorFollower :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
  public: void Configure(const gz::sim::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         gz::sim::EntityComponentManager &_ecm,
                         gz::sim::EventManager &) override
  {
    this->actorEntity = _entity;

    if (_sdf->HasElement("actor_name"))
    {
      this->actorName = _sdf->Get<std::string>("actor_name");
    }

    if (_sdf->HasElement("target"))
    {
      this->targetName = _sdf->Get<std::string>("target");
    }

    if (_sdf->HasElement("animation_walk"))
    {
      this->animWalkName = _sdf->Get<std::string>("animation_walk");
    }

    if (_sdf->HasElement("animation_stand"))
    {
      this->animStandName = _sdf->Get<std::string>("animation_stand");
    }

    if (_sdf->HasElement("z_offset"))
    {
      this->zOffset = _sdf->Get<double>("z_offset");
    }

    if (_sdf->HasElement("animation_x_vel"))
    {
      this->animSpeedScale = _sdf->Get<double>("animation_x_vel");
    }

    // 1. Determine initial pose from actor or target collider to prevent spawning at (0,0,0)
    auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->actorEntity);
    gz::math::Pose3d initialPose = gz::math::Pose3d::Zero;
    if (poseComp && poseComp->Data() != gz::math::Pose3d::Zero)
    {
      initialPose = poseComp->Data();
    }
    else
    {
      this->targetEntity = _ecm.EntityByComponents(
          gz::sim::components::Name(this->targetName));
      if (this->targetEntity != gz::sim::kNullEntity)
      {
        if (auto targetPose = _ecm.Component<gz::sim::components::Pose>(this->targetEntity))
        {
          this->lastTargetPose = targetPose->Data();
          this->hasTargetPose = true;
          initialPose = targetPose->Data();
          initialPose.Pos().Z() += this->zOffset;
        }
      }
    }

    // Zero out actor base Pose component so TrajectoryPose directly
    // represents world coordinates without relative offset doubling.
    if (!poseComp)
    {
      _ecm.CreateComponent(this->actorEntity,
          gz::sim::components::Pose(gz::math::Pose3d::Zero));
    }
    else
    {
      *poseComp = gz::sim::components::Pose(gz::math::Pose3d::Zero);
    }
    _ecm.SetChanged(this->actorEntity,
        gz::sim::components::Pose::typeId,
        gz::sim::ComponentState::OneTimeChange);

    // 2. Initialize AnimationName to stand (idle)
    this->currentAnimName = this->animStandName;
    auto animNameComp = _ecm.Component<gz::sim::components::AnimationName>(this->actorEntity);
    if (!animNameComp)
    {
      _ecm.CreateComponent(this->actorEntity,
          gz::sim::components::AnimationName(this->currentAnimName));
    }
    else
    {
      *animNameComp = gz::sim::components::AnimationName(this->currentAnimName);
    }
    _ecm.SetChanged(this->actorEntity,
        gz::sim::components::AnimationName::typeId,
        gz::sim::ComponentState::OneTimeChange);

    // 3. Initialize AnimationTime
    auto animTimeComp = _ecm.Component<gz::sim::components::AnimationTime>(this->actorEntity);
    if (!animTimeComp)
    {
      _ecm.CreateComponent(this->actorEntity,
          gz::sim::components::AnimationTime(std::chrono::steady_clock::duration::zero()));
    }
    else
    {
      *animTimeComp = gz::sim::components::AnimationTime(std::chrono::steady_clock::duration::zero());
    }
    _ecm.SetChanged(this->actorEntity,
        gz::sim::components::AnimationTime::typeId,
        gz::sim::ComponentState::OneTimeChange);

    // 4. Initialize TrajectoryPose with true initial world pose
    auto trajPoseComp = _ecm.Component<gz::sim::components::TrajectoryPose>(this->actorEntity);
    if (!trajPoseComp)
    {
      _ecm.CreateComponent(this->actorEntity,
          gz::sim::components::TrajectoryPose(initialPose));
    }
    else
    {
      *trajPoseComp = gz::sim::components::TrajectoryPose(initialPose);
    }
    _ecm.SetChanged(this->actorEntity,
        gz::sim::components::TrajectoryPose::typeId,
        gz::sim::ComponentState::OneTimeChange);

    this->currentSmoothedPose = initialPose;
    this->hasSmoothedPose = (initialPose != gz::math::Pose3d::Zero);

    // Subscribe to topics if actorName is available
    if (this->actorName.empty())
    {
      auto nameComp = _ecm.Component<gz::sim::components::Name>(this->actorEntity);
      if (nameComp && !nameComp->Data().empty())
      {
        this->actorName = nameComp->Data();
      }
    }

    if (!this->actorName.empty())
    {
      this->SubscribeTopics(this->actorName);
    }

    gzmsg << "[GazeboActorFollower] Initialized for actor [" << this->actorEntity
          << "] tracking target [" << this->targetName << "] (z_offset: "
          << this->zOffset << "m)" << std::endl;
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                         gz::sim::EntityComponentManager &_ecm) override
  {
    // Ensure topics are subscribed if not yet done during Configure
    if (!this->subscribed)
    {
      std::string aName = this->actorName;
      if (aName.empty())
      {
        auto nameComp = _ecm.Component<gz::sim::components::Name>(this->actorEntity);
        if (nameComp && !nameComp->Data().empty())
          aName = nameComp->Data();
      }
      if (aName.empty() && !this->targetName.empty())
      {
        if (this->targetName.size() > 10 && this->targetName.rfind("_collision") == this->targetName.size() - 10)
          aName = this->targetName.substr(0, this->targetName.size() - 10);
        else
          aName = this->targetName;
      }
      if (!aName.empty())
      {
        this->SubscribeTopics(aName);
      }
    }

    // 0. Auto-destruction check: explicit topic signal OR target collider removal
    bool shouldDelete = false;

    if (this->initialized)
    {
      if (this->deleteRequested.load())
      {
        shouldDelete = true;
      }
      else if (this->targetEntity != gz::sim::kNullEntity && !_ecm.HasEntity(this->targetEntity))
      {
        shouldDelete = true;
      }
    }

    if (shouldDelete)
    {
      gzmsg << "[GazeboActorFollower] Deleting actor entity [" << this->actorEntity
            << "] (target: " << this->targetName << ")." << std::endl;

      // Teleport deep underground instantly so visual mesh vanishes from camera/GUI view
      gz::math::Pose3d hiddenPose(0, 0, -1000.0, 0, 0, 0);
      auto trajPoseComp = _ecm.Component<gz::sim::components::TrajectoryPose>(this->actorEntity);
      if (trajPoseComp)
      {
        *trajPoseComp = gz::sim::components::TrajectoryPose(hiddenPose);
        _ecm.SetChanged(this->actorEntity,
            gz::sim::components::TrajectoryPose::typeId,
            gz::sim::ComponentState::OneTimeChange);
      }

      _ecm.RequestRemoveEntity(this->actorEntity);
      return;
    }

    if (_info.paused)
      return;

    // Find target entity by name if not cached
    if (this->targetEntity == gz::sim::kNullEntity)
    {
      this->targetEntity = _ecm.EntityByComponents(
          gz::sim::components::Name(this->targetName));
      if (this->targetEntity == gz::sim::kNullEntity)
        return;
    }

    // Retrieve target pose (from WorldPoseCmd, active direct topic callback, or collider Pose component)
    {
      std::lock_guard<std::mutex> lock(this->poseMutex);
      if (auto cmd = _ecm.Component<gz::sim::components::WorldPoseCmd>(this->targetEntity))
      {
        this->lastTargetPose = cmd->Data();
        this->hasTargetPose = true;
      }
      else
      {
        bool useTopic = this->hasTopicPose &&
            (std::chrono::duration<double>(std::chrono::steady_clock::now() - this->lastTopicTime).count() < 0.3);
        if (!useTopic)
        {
          if (auto pose = _ecm.Component<gz::sim::components::Pose>(this->targetEntity))
          {
            this->lastTargetPose = pose->Data();
            this->hasTargetPose = true;
          }
        }
      }
    }

    if (!this->hasTargetPose)
      return;

    gz::math::Pose3d targetPose;
    {
      std::lock_guard<std::mutex> lock(this->poseMutex);
      targetPose = this->lastTargetPose;
    }

    // Align actor pose with collider target, adding Z offset for feet-to-ground alignment
    gz::math::Pose3d actorPose = targetPose;
    actorPose.Pos().Z() += this->zOffset;

    // Enforce base Pose remains zero so TrajectoryPose is absolute world coordinates
    auto basePoseComp = _ecm.Component<gz::sim::components::Pose>(this->actorEntity);
    if (basePoseComp && basePoseComp->Data() != gz::math::Pose3d::Zero)
    {
      *basePoseComp = gz::sim::components::Pose(gz::math::Pose3d::Zero);
      _ecm.SetChanged(this->actorEntity,
          gz::sim::components::Pose::typeId,
          gz::sim::ComponentState::OneTimeChange);
    }

    double dt = std::chrono::duration<double>(_info.dt).count();
    if (dt <= 1e-6)
      return;

    // 1. Exponential rate smoothing between 20 Hz discrete ROS updates and 250 Hz Gazebo physics
    if (!this->hasSmoothedPose || this->currentSmoothedPose == gz::math::Pose3d::Zero)
    {
      this->currentSmoothedPose = actorPose;
      this->hasSmoothedPose = true;
    }
    else
    {
      double dist = (actorPose.Pos() - this->currentSmoothedPose.Pos()).Length();
      if (dist > 1.5)
      {
        // Immediate snap on teleport / initial spawn / large reset
        this->currentSmoothedPose = actorPose;
      }
      else
      {
        // Exponential smoothing: factor = 1.0 - exp(-lambda * dt)
        // lambda = 28.0 gives silky smooth motion with ~30ms response time
        double smoothFactor = std::min(1.0, 1.0 - std::exp(-28.0 * dt));
        gz::math::Vector3d smoothPos = this->currentSmoothedPose.Pos() +
            smoothFactor * (actorPose.Pos() - this->currentSmoothedPose.Pos());
        gz::math::Quaterniond smoothRot = gz::math::Quaterniond::Slerp(
            smoothFactor, this->currentSmoothedPose.Rot(), actorPose.Rot());

        // Snap micro-residuals when stopped to prevent creeping
        if (dist < 0.0005)
        {
          smoothPos = actorPose.Pos();
          smoothRot = actorPose.Rot();
        }

        this->currentSmoothedPose = gz::math::Pose3d(smoothPos, smoothRot);
      }
    }

    // 2. Update TrajectoryPose with smoothed pose and notify ECM change tracker for SceneBroadcaster
    auto trajPoseComp = _ecm.Component<gz::sim::components::TrajectoryPose>(this->actorEntity);
    if (!trajPoseComp)
    {
      _ecm.CreateComponent(this->actorEntity,
          gz::sim::components::TrajectoryPose(this->currentSmoothedPose));
    }
    else
    {
      *trajPoseComp = gz::sim::components::TrajectoryPose(this->currentSmoothedPose);
    }
    _ecm.SetChanged(this->actorEntity,
        gz::sim::components::TrajectoryPose::typeId,
        gz::sim::ComponentState::OneTimeChange);

    if (!this->initialized)
    {
      this->prevPose = actorPose;
      this->lastMoveTime = std::chrono::steady_clock::duration::zero();
      this->initialized = true;
      return;
    }

    // Detect movement (horizontal translation in XY and yaw rotation)
    // Isolates horizontal motion to eliminate vertical terrain chatter on slopes.
    double dx = actorPose.Pos().X() - this->prevPose.Pos().X();
    double dy = actorPose.Pos().Y() - this->prevPose.Pos().Y();
    double horizDist = std::hypot(dx, dy);
    double rotDiff = std::abs(actorPose.Rot().Yaw() - this->prevPose.Rot().Yaw());
    if (rotDiff > M_PI)
      rotDiff = 2.0 * M_PI - rotDiff;

    // Movement threshold: only intentional horizontal translation or yaw turning
    if (horizDist > 0.008 || rotDiff > 0.02)
    {
      this->lastMoveTime = _info.simTime;
    }

    // Check if moving (grace window of 0.20s bridges 20Hz ROS update interval)
    double timeSinceMove = std::chrono::duration<double>(_info.simTime - this->lastMoveTime).count();
    bool isMoving = (this->lastMoveTime > std::chrono::steady_clock::duration::zero()) &&
                    (timeSinceMove < 0.20);

    std::string activeCmd = "";
    {
      std::lock_guard<std::mutex> lock(this->animMutex);
      activeCmd = this->commandedAnim;
    }

    // Determine target animation:
    // When an animation command has been received from ROS 2, it is strictly authoritative.
    // It will never be overridden by slope physics jitter or false movement detection.
    // Only fall back to autonomous movement detection if no ROS command has been set yet.
    std::string targetAnim = this->animStandName;
    if (!activeCmd.empty())
    {
      targetAnim = activeCmd;
    }
    else
    {
      targetAnim = isMoving ? this->animWalkName : this->animStandName;
    }

    // 2. Switch animation clip if changed
    if (targetAnim != this->currentAnimName)
    {
      this->currentAnimName = targetAnim;
      this->animTime = std::chrono::steady_clock::duration::zero();

      auto animNameComp = _ecm.Component<gz::sim::components::AnimationName>(this->actorEntity);
      if (!animNameComp)
      {
        _ecm.CreateComponent(this->actorEntity,
            gz::sim::components::AnimationName(this->currentAnimName));
      }
      else
      {
        *animNameComp = gz::sim::components::AnimationName(this->currentAnimName);
      }
      _ecm.SetChanged(this->actorEntity,
          gz::sim::components::AnimationName::typeId,
          gz::sim::ComponentState::OneTimeChange);
    }

    // 3. Advance AnimationTime smoothly
    if (targetAnim == this->animWalkName)
    {
      // Advance walk animation smoothly paced to nominal stride rate
      this->animTime += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(dt * this->animSpeedScale));
    }
    else
    {
      // Advance idle/gesture animation at normal real-time pace (1.0x)
      this->animTime += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(dt));
    }

    auto animTimeComp = _ecm.Component<gz::sim::components::AnimationTime>(this->actorEntity);
    if (!animTimeComp)
    {
      _ecm.CreateComponent(this->actorEntity,
          gz::sim::components::AnimationTime(this->animTime));
    }
    else
    {
      *animTimeComp = gz::sim::components::AnimationTime(this->animTime);
    }
    _ecm.SetChanged(this->actorEntity,
        gz::sim::components::AnimationTime::typeId,
        gz::sim::ComponentState::OneTimeChange);

    this->prevPose = actorPose;
  }

  private: gz::sim::Entity actorEntity{gz::sim::kNullEntity};
  private: gz::sim::Entity targetEntity{gz::sim::kNullEntity};
  private: std::string actorName{""};
  private: std::string targetName{"human_worker_collision"};
  private: std::string animWalkName{"walk"};
  private: std::string animStandName{"stand"};
  private: std::string currentAnimName{""};
  private: std::string commandedAnim{""};
  private: std::string targetCustomAnim{""};
  private: std::mutex animMutex;
  private: double zOffset{0.86};
  private: double animSpeedScale{1.5};
  private: gz::math::Pose3d lastTargetPose{gz::math::Pose3d::Zero};
  private: bool hasTargetPose{false};
  private: std::mutex poseMutex;
  private: gz::math::Pose3d currentSmoothedPose{gz::math::Pose3d::Zero};
  private: bool hasSmoothedPose{false};
  private: gz::math::Pose3d prevPose{gz::math::Pose3d::Zero};
  private: std::chrono::steady_clock::duration animTime{0};
  private: std::chrono::steady_clock::duration lastMoveTime{0};
  private: bool initialized{false};
  private: bool subscribed{false};
  private: gz::transport::Node transportNode;
  private: std::atomic<bool> deleteRequested{false};
  private: bool hasTopicPose{false};
  private: std::chrono::steady_clock::time_point lastTopicTime;
  private: void OnPoseCmd(const gz::msgs::Pose &_msg)
  {
    std::lock_guard<std::mutex> lock(this->poseMutex);
    this->lastTargetPose = gz::math::Pose3d(
        _msg.position().x(), _msg.position().y(), _msg.position().z(),
        _msg.orientation().w(), _msg.orientation().x(), _msg.orientation().y(), _msg.orientation().z());
    this->hasTargetPose = true;
    this->hasTopicPose = true;
    this->lastTopicTime = std::chrono::steady_clock::now();
  }
  private: void OnDeleteMsg(const gz::msgs::Boolean &_msg)
  {
    if (_msg.data() && this->initialized)
    {
      this->deleteRequested.store(true);
    }
  }
  private: void OnAnimationCmd(const gz::msgs::StringMsg &_msg)
  {
    std::lock_guard<std::mutex> lock(this->animMutex);
    if (!_msg.data().empty())
    {
      std::string anim = _msg.data();
      if (anim == "idle")
      {
        anim = this->animStandName;
      }
      this->commandedAnim = anim;
      if (anim != this->animWalkName)
      {
        this->targetCustomAnim = anim;
      }
      gzmsg << "[GazeboActorFollower] Recibido comando animacion: [" << _msg.data()
            << "] -> commandedAnim: [" << this->commandedAnim
            << "], targetCustomAnim: [" << this->targetCustomAnim << "]" << std::endl;
    }
  }
  private: void SubscribeTopics(const std::string &_name)
  {
    if (this->subscribed || _name.empty())
      return;

    this->actorName = _name;

    std::string delTopic = "/actor/" + _name + "/delete";
    this->transportNode.Subscribe(delTopic, &ActorFollower::OnDeleteMsg, this);

    std::string animTopic = "/actor/" + _name + "/animation_cmd";
    this->transportNode.Subscribe(animTopic, &ActorFollower::OnAnimationCmd, this);

    std::string poseTopic = "/actor/" + _name + "/pose_cmd";
    this->transportNode.Subscribe(poseTopic, &ActorFollower::OnPoseCmd, this);

    // Subscribirse tambien con el nombre del modelo fisico de colision
    if (!this->targetName.empty() && this->targetName != _name)
    {
      this->transportNode.Subscribe("/actor/" + this->targetName + "/animation_cmd", &ActorFollower::OnAnimationCmd, this);
      this->transportNode.Subscribe("/actor/" + this->targetName + "/delete", &ActorFollower::OnDeleteMsg, this);
      this->transportNode.Subscribe("/actor/" + this->targetName + "/pose_cmd", &ActorFollower::OnPoseCmd, this);

      if (this->targetName.size() > 10 && this->targetName.rfind("_collision") == this->targetName.size() - 10)
      {
        std::string baseCol = this->targetName.substr(0, this->targetName.size() - 10);
        if (baseCol != _name)
        {
          this->transportNode.Subscribe("/actor/" + baseCol + "/animation_cmd", &ActorFollower::OnAnimationCmd, this);
          this->transportNode.Subscribe("/actor/" + baseCol + "/delete", &ActorFollower::OnDeleteMsg, this);
          this->transportNode.Subscribe("/actor/" + baseCol + "/pose_cmd", &ActorFollower::OnPoseCmd, this);
        }
      }
    }

    this->subscribed = true;
    gzmsg << "[GazeboActorFollower] Suscrito a topicos para actor [" << _name << "] y colisionador [" << this->targetName << "]" << std::endl;
  }
};

}  // namespace gazebo_actors

GZ_ADD_PLUGIN(gazebo_actors::ActorFollower,
              gz::sim::System,
              gazebo_actors::ActorFollower::ISystemConfigure,
              gazebo_actors::ActorFollower::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gazebo_actors::ActorFollower, "gazebo_actors::ActorFollower")
GZ_ADD_PLUGIN_ALIAS(gazebo_actors::ActorFollower, "wildboar::VineyardActorFollower")

