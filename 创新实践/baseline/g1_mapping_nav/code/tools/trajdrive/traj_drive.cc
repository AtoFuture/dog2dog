// traj_drive.cc — gz-sim (Ignition Fortress, gz-sim6) system plugin.
// Drives a single-rigid-body model (no wheels/joints) along a waypoint loop with a
// pure-pursuit controller + reverse-at-end (so it stays in a narrow lane instead of
// doing 180 turns). The body stays *dynamic* -> it RESPECTS COLLISIONS and can be
// pushed by other bodies.
//
// SDF usage (inside a <model>):
//   <plugin filename="libTrajDrive.so" name="traj_drive::TrajDrive">
//     <link>link</link>
//     <speed>0.5</speed>
//     <kp>2.2</kp>
//     <lookahead>1.5</lookahead>
//     <loop>true</loop>
//     <waypoint>-6.5 -6.45 0</waypoint>
//     <waypoint>1.5 -6.45 0</waypoint>
//   </plugin>
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/AngularVelocityCmd.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/common/Console.hh>
#include <sdf/Element.hh>

namespace traj_drive
{
class TrajDrive
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate
{
public:
  void Configure(const ignition::gazebo::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::EventManager &) override
  {
    ignition::gazebo::Model model(_entity);
    if (!model.Valid(_ecm))
    {
      ignerr << "[TrajDrive] plugin must be attached to a model\n";
      return;
    }
    this->model = model;
    sdf::ElementPtr sdf = _sdf->Clone();
    if (sdf->HasElement("link"))
      this->linkName = sdf->Get<std::string>("link");
    if (sdf->HasElement("speed"))
      this->speed = sdf->Get<double>("speed");
    if (sdf->HasElement("kp"))
      this->kp = sdf->Get<double>("kp");
    if (sdf->HasElement("lookahead"))
      this->la = sdf->Get<double>("lookahead");
    if (sdf->HasElement("loop"))
      this->loop = sdf->Get<bool>("loop");

    for (auto wp = sdf->GetElement("waypoint"); wp;
         wp = wp->GetNextElement("waypoint"))
    {
      auto v = wp->Get<ignition::math::Vector3d>();
      this->wps.emplace_back(v.X(), v.Y());
    }
    if (this->wps.size() < 2u)
      ignerr << "[TrajDrive] model [" << model.Name(_ecm)
             << "] needs >=2 <waypoint>\n";
  }

  void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                 ignition::gazebo::EntityComponentManager &_ecm) override
  {
    if (_info.paused || this->wps.size() < 2u)
      return;
    const auto linkEntity = this->model.LinkByName(_ecm, this->linkName);
    if (linkEntity == ignition::gazebo::kNullEntity)
      return;
    const auto pose = ignition::gazebo::worldPose(linkEntity, _ecm);
    const double px = pose.Pos().X(), py = pose.Pos().Y();
    const double yaw = pose.Rot().Yaw();

    // current segment A->B
    const std::size_t n = this->wps.size();
    const auto &A = this->wps[this->seg];
    const auto &B = this->wps[(this->seg + 1) % n];
    double sx = B.first - A.first, sy = B.second - A.second;
    const double dl = std::hypot(sx, sy);
    if (dl < 1e-6) { this->seg = (this->seg + 1) % n; return; }
    const double ux = sx / dl, uy = sy / dl;
    double proj = (px - A.first) * ux + (py - A.second) * uy;
    if (proj > dl - this->la * 0.5)
    {
      if (this->seg + 1u < n || this->loop)
        this->seg = (this->seg + 1) % n;
    }
    proj = std::max(0.0, std::min(dl, proj));
    const double lp = std::min(dl, proj + this->la);   // look-ahead along segment
    const double tx = A.first + ux * lp;
    const double ty = A.second + uy * lp;

    double err = std::atan2(ty - py, tx - px) - yaw;
    while (err > M_PI)  err -= 2.0 * M_PI;
    while (err < -M_PI) err += 2.0 * M_PI;

    double v, wz;
    if (std::fabs(err) <= M_PI / 2.0)
    {
      v = this->speed;
      wz = this->kp * err;
    }
    else                       // drive backward, keep in lane
    {
      v = -this->speed;
      wz = this->kp * (err - std::copysign(M_PI, err));
    }
    wz = std::clamp(wz, -1.2, 1.2);

    if ((this->dbg++ % 1500) == 0)
      ignerr << "[TrajDrive] " << this->model.Name(_ecm) << " seg=" << this->seg
             << " pos=" << px << "," << py << " yaw=" << yaw
             << " v=" << v << " wz=" << wz << std::endl;

    _ecm.SetComponentData<ignition::gazebo::components::LinearVelocityCmd>(
        linkEntity, ignition::math::Vector3d(v, 0.0, 0.0));
    _ecm.SetComponentData<ignition::gazebo::components::AngularVelocityCmd>(
        linkEntity, ignition::math::Vector3d(0.0, 0.0, wz));
  }

private:
  ignition::gazebo::Model model{ignition::gazebo::kNullEntity};
  std::string linkName{"link"};
  double speed{0.5};
  double kp{2.2};
  double la{1.5};
  bool loop{true};
  std::size_t seg{0u};
  int dbg{0};
  std::vector<std::pair<double, double>> wps;
};
}  // namespace traj_drive

IGNITION_ADD_PLUGIN(traj_drive::TrajDrive,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPreUpdate)
