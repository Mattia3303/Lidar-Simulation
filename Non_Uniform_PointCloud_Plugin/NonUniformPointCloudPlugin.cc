#include <ignition/plugin/Register.hh>
#include <ignition/sensors/GpuLidarSensor.hh>
#include <ignition/sensors/Sensor.hh>
#include <ignition/transport/Node.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Sensor.hh>
#include <ignition/gazebo/EntityComponentManager.hh>

#include <cmath>
#include <vector>

namespace custom_plugins
{
  class NonUniformPointCloudPlugin :
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPostUpdate
  {
  public:
    // Configure the plugin
    void Configure(const ignition::gazebo::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &/*_sdf*/,
                   ignition::gazebo::EntityComponentManager &/*_ecm*/,
                   ignition::gazebo::EventManager &/*_eventMgr*/) override
    {
      ignmsg << "NonUniformPointCloudPlugin configured." << std::endl;

      // Subscribe to the original LiDAR scan topic
      this->node.Subscribe("/lidar/scan/points", &NonUniformPointCloudPlugin::OnLidarScan, this);
      // Publisher for the modified point cloud
      this->pub = this->node.Advertise<ignition::msgs::PointCloudPacked>("/lidar/non_uniform_scan/points");
    }

    // PostUpdate callback if needed for runtime updates
    void PostUpdate(const ignition::gazebo::UpdateInfo &/*_info*/,
                    const ignition::gazebo::EntityComponentManager &/*_ecm*/) override
    {
    }

  private:
  void OnLidarScan(const ignition::msgs::PointCloudPacked &_msg)
  {
    ignition::msgs::PointCloudPacked modifiedMsg;

      // Copy the header and fields from the original message
      modifiedMsg.mutable_header()->CopyFrom(_msg.header());
      for (const auto &field : _msg.field())
      {
        auto newField = modifiedMsg.add_field();
        newField->CopyFrom(field);
      }
      modifiedMsg.set_is_bigendian(_msg.is_bigendian());
      modifiedMsg.set_point_step(_msg.point_step());
      modifiedMsg.set_row_step(0); // Will update later
      modifiedMsg.set_height(1);
      modifiedMsg.set_is_dense(true);

      // Parse the incoming point cloud data
      size_t numPoints = _msg.data().size() / _msg.point_step();
      modifiedMsg.mutable_data()->reserve(_msg.data().size());

      // For simplicity, assume that angles are uniformly spaced horizontally in the raw LiDAR scan
      std::vector<double> angles(numPoints);
      for (size_t i = 0; i < numPoints; ++i)
      {
        // Map index to an angular value (from -1.5708 to 1.5708 for [-90, 90] degrees)
        double angle = -1.5708 + (3.1416 * i / numPoints);
        angles[i] = angle;
      }

      // Filter points to create a denser distribution near the center
      for (size_t i = 0; i < numPoints; ++i)
      {
        double angle = angles[i];

        // Apply a density function: higher weight near the center (e.g., Gaussian-like distribution)
        double weight = std::exp(-std::pow(angle / 0.7854, 2)); // Narrower spread for more density near 0

        // Randomly decide whether to keep this point based on weight
        if ((double)rand() / RAND_MAX < weight)
        {
          const uint8_t *start = reinterpret_cast<const uint8_t *>(&_msg.data()[i * _msg.point_step()]);
          modifiedMsg.mutable_data()->insert(modifiedMsg.mutable_data()->end(), start, start + _msg.point_step());
        }
      }

      // Update the row_step and width in the modified message
      size_t modifiedNumPoints = modifiedMsg.data().size() / modifiedMsg.point_step();
      modifiedMsg.set_row_step(modifiedNumPoints * modifiedMsg.point_step());
      modifiedMsg.set_width(modifiedNumPoints);

      // Publish the modified point cloud
      this->pub.Publish(modifiedMsg);
  }


    ignition::transport::Node node;
    ignition::transport::Node::Publisher pub;
  };
}

IGNITION_ADD_PLUGIN(custom_plugins::NonUniformPointCloudPlugin,
                    ignition::gazebo::System,
                    custom_plugins::NonUniformPointCloudPlugin::ISystemConfigure,
                    custom_plugins::NonUniformPointCloudPlugin::ISystemPostUpdate)
