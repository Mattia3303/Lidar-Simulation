#include <ignition/plugin1/Register.hh>
#include <ignition/sensors6/GpuLidarSensor.hh>
#include <ignition/sensors6/Sensor.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/laserscan.pb.h>

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
      this->node.Subscribe("/lidar/scan", &NonUniformPointCloudPlugin::OnLidarScan, this);
      // Publisher for the modified point cloud
      this->pub = this->node.Advertise<ignition::msgs::LaserScan>("/lidar/non_uniform_scan");
    }

    // PostUpdate callback if needed for runtime updates
    void PostUpdate(const ignition::gazebo::UpdateInfo &/*_info*/,
                    const ignition::gazebo::EntityComponentManager &/*_ecm*/) override
    {
    }

  private:
    void OnLidarScan(const ignition::msgs::LaserScan &_msg)
    {
      ignition::msgs::LaserScan modifiedMsg = _msg;

      int numPoints = _msg.ranges_size();
      std::vector<float> newRanges(numPoints);

      // Apply a Gaussian weighting for non-uniform density
      double center = numPoints / 2.0;  // Center index
      double sigma = center / 3.0;     // Spread for Gaussian distribution

      for (int i = 0; i < numPoints; ++i)
      {
        double weight = std::exp(-std::pow((i - center) / sigma, 2) / 2.0);
        newRanges[i] = _msg.ranges(i) * weight; // Scale ranges based on weight
      }

      // Set the modified ranges
      for (int i = 0; i < numPoints; ++i)
      {
        modifiedMsg.set_ranges(i, newRanges[i]);
      }

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
