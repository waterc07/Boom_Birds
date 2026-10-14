// 用物理力/力矩驱动传感器平台，保留 IMU 与运动加速度的一致性。
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <sstream>

namespace boom_birds {
class SensorMotion final : public gz::sim::System,
                           public gz::sim::ISystemConfigure,
                           public gz::sim::ISystemPreUpdate {
public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override {
    link = gz::sim::Link(gz::sim::Model(entity).LinkByName(ecm, "imu_link"));
    link.EnableVelocityChecks(ecm, true);
    std::istringstream amplitudes(sdf->Get<std::string>("amplitudes"));
    std::istringstream frequencies(sdf->Get<std::string>("frequencies"));
    for (int i = 0; i < 6; ++i) {
      amplitudes >> amplitude[i];
      frequencies >> frequency[i];
    }
    start = sdf->Get<double>("start_s");
    stop = sdf->Get<double>("stop_s");
    ramp = sdf->Get<double>("ramp_s");
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (info.paused)
      return;
    const auto linear = link.WorldLinearVelocity(ecm);
    const auto angular = link.WorldAngularVelocity(ecm);
    if (!linear || !angular)
      return;
    const double t = std::chrono::duration<double>(info.simTime).count();
    std::array<double, 6> target{};
    if (t >= start && t < stop) {
      const double factor =
          std::min({(t - start) / ramp, 1.0, (stop - t) / ramp});
      for (int i = 0; i < 6; ++i)
        target[i] =
            factor * amplitude[i] * std::sin(frequency[i] * (t - start));
    }
    const auto rotation = gz::sim::worldPose(link.Entity(), ecm).Rot();
    const gz::math::Vector3d velocity(target[0], target[1], target[2]);
    const gz::math::Vector3d bodyRate(target[3], target[4], target[5]);
    // 模型质量 1 kg、各轴惯量 0.1 kg m²；此增益仅属于模拟运动平台。
    const auto force = 8.0 * (velocity - *linear);
    const auto torque = 0.8 * (rotation.RotateVector(bodyRate) - *angular);
    link.AddWorldWrench(ecm, force, torque);
  }

private:
  gz::sim::Link link;
  std::array<double, 6> amplitude{}, frequency{};
  double start{}, stop{}, ramp{};
};
} // namespace boom_birds
GZ_ADD_PLUGIN(boom_birds::SensorMotion, gz::sim::System,
              boom_birds::SensorMotion::ISystemConfigure,
              boom_birds::SensorMotion::ISystemPreUpdate)
