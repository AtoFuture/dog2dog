// traj_drive.cc — gz-sim (Ignition Fortress, gz-sim6) system plugin.
// Path -> cmd_vel publisher. Attach to a wheeled model that ALSO has the official
// ignition::gazebo::systems::AckermannSteering plugin (topic /model/<name>/cmd_vel).
// It follows a closed polyline with a pure-pursuit controller and publishes Twist
// (linear.x = speed, angular.z = yaw rate). This gives CLEAN, drift-free driving
// (the Ackermann system does the wheel-level control) while the car stays a real
// dynamic body (collides, and can be pushed).
//
// SDF:
//   <plugin filename="libTrajDrive.so" name="traj_drive::TrajDrive">
//     <link>chassis</link>
//     <speed>0.6</speed>
//     <kp>1.5</kp>
//     <lookahead>2.0</lookahead>
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
#include <ignition/plugin/Register.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/common/Console.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/twist.pb.h>
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
    const std::string topic = "/model/" + model.Name(_ecm) + "/cmd_vel";
    this->pub = this->node.Advertise<ignition::msgs::Twist>(topic);
    if (this->wps.size() < 2u)
      ignerr << "[TrajDrive] model [" << model.Name(_ecm) << "] needs >=2 <waypoint>\n";
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
    const double px = pose.Pos().X(), py = pose.Pos().Y(), yaw = pose.Rot().Yaw();

    const std::size_t n = this->wps.size();
    const auto &A = this->wps[this->seg];
    const auto &B = this->wps[(this->seg + 1) % n];
    double sx = B.first - A.first, sy = B.second - A.second;
    const double dl = std::hypot(sx, sy);
    if (dl < 1e-6) { this->seg = (this->seg + 1) % n; return; }
    const double ux = sx / dl, uy = sy / dl;
    double proj = (px - A.first) * ux + (py - A.second) * uy;
    if (proj > dl - this->la * 0.5)
      this->seg = (this->seg + 1) % n;
    proj = std::max(0.0, std::min(dl, proj));
    const double lp = std::min(dl, proj + this->la);
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
    else
    {
      v = -this->speed;
      wz = this->kp * (err - std::copysign(M_PI, err));
    }
    wz = std::clamp(wz, -1.0, 1.0);

    ignition::msgs::Twist msg;
    msg.mutable_linear()->set_x(v);
    msg.mutable_angular()->set_z(wz);
    this->pub.Publish(msg);

    if ((this->dbg++ % 1500) == 0)
      ignerr << "[TrajDrive] " << this->model.Name(_ecm) << " seg=" << this->seg
             << " pos=" << px << "," << py << " yaw=" << yaw
             << " v=" << v << " wz=" << wz << std::endl;
  }

private:
  ignition::gazebo::Model model{ignition::gazebo::kNullEntity};
  ignition::transport::Node node;
  ignition::transport::Node::Publisher pub;
  std::string linkName{"chassis"};
  double speed{0.6};
  double kp{1.5};
  double la{2.0};
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
