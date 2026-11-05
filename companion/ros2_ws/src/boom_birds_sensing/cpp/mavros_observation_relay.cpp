#include <array>
#include <algorithm>
#include <memory>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/mavlink.hpp>

class ObservationRelay : public rclcpp::Node {
 public:
  using Mavlink = mavros_msgs::msg::Mavlink;
  ObservationRelay() : Node("mavros_observation_relay") {
    const auto input = declare_parameter<std::string>("input_topic", "/uas1/mavlink_source");
    const auto output = declare_parameter<std::string>("output_topic", "/boom_birds/mavros/control_observations");
    auto qos = rclcpp::SensorDataQoS().keep_last(1000);
    publisher = create_publisher<Mavlink>(output, qos);
    for (const auto id : {30u, 32u, 230u}) {
      latestPublishers[id] = create_publisher<Mavlink>(output + "/latest_" + std::to_string(id),
                                             rclcpp::SensorDataQoS().keep_last(1));
    }
    for (const auto id : {105u, 132u}) {
      sensorPublishers[id] = create_publisher<Mavlink>(output + "/sensor_" + std::to_string(id), qos);
    }
    subscription = create_subscription<Mavlink>(input, qos, [this](Mavlink::ConstSharedPtr message) {
      if (message->framing_status != 1) return;
      const auto sensor = sensorPublishers.find(message->msgid);
      if (sensor != sensorPublishers.end()) {
        sensor->second->publish(*message);
        return;
      }
      static constexpr std::array<uint32_t, 10> ids{0, 1, 30, 32, 77, 230, 245, 331, 436, 253};
      if (message->framing_status == 1 && std::find(ids.begin(), ids.end(), message->msgid) != ids.end()) {
        const auto latest = latestPublishers.find(message->msgid);
        if (latest != latestPublishers.end()) latest->second->publish(*message);
        else publisher->publish(*message);
      }
    });
  }
 private:
  rclcpp::Publisher<Mavlink>::SharedPtr publisher;
  std::unordered_map<uint32_t, rclcpp::Publisher<Mavlink>::SharedPtr> latestPublishers, sensorPublishers;
  rclcpp::Subscription<Mavlink>::SharedPtr subscription;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObservationRelay>());
  rclcpp::shutdown();
  return 0;
}
