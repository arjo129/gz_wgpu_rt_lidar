#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/common/Console.hh>
#include <sdf/Element.hh>
#include <string>

namespace sensorconfig
{
  /// \brief A plugin that reads sensor configuration from SDF and prints it.
  class SensorConfig :
    public gz::sim::System,
    public gz::sim::ISystemConfigure
  {
    /// \brief Constructor
    public: SensorConfig();
    /// \brief Destructor
    public: ~SensorConfig();

    /// \brief Configure the sensor from SDF.
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &) override;

  };

  SensorConfig::SensorConfig() {}
  SensorConfig::~SensorConfig()
  {
    // Destructor logic if needed}
    printf("SensorConfig destructor called.\n");
  }
    void SensorConfig::Configure(const gz::sim::Entity &_entity,
                                 const std::shared_ptr<const sdf::Element> &_sdf,
                                 gz::sim::EntityComponentManager &_ecm,
                                 gz::sim::EventManager &)
    {
        if (!_sdf)
        {
        gzerr << "SensorConfig SDF element is null." << std::endl;
        return;
        }
        // Print sensor configuration from SDF
        std::string sensorType = _sdf->Get<std::string>("type");
        gzmsg << std::endl << "****************   " << sensorType <<"   ****************"<< std::endl <<std::endl;
        gzmsg << "THIS IS SENSOR PLUGIN TEST FOR: " << sensorType << std::endl <<std::endl;
        // Additional configuration logic can be added here
    }
}

GZ_ADD_PLUGIN(
  sensorconfig::SensorConfig,
  gz::sim::System,
  sensorconfig::SensorConfig::ISystemConfigure)
