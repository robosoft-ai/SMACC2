// Copyright 2021 RobosoftAI Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/


#ifndef GAZEBO_SMACC_PLUGINS_FLASHLIGHTPLUGIN_HH_
#define GAZEBO_SMACC_PLUGINS_FLASHLIGHTPLUGIN_HH_

#include <memory>
#include <string>

#include <ignition/math/Color.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/int8.hpp>
#include <rclcpp/rclcpp.hpp>


namespace smacc2
{
  using namespace gazebo;

  // forward declaration
  class FlashLightSettingPrivate;

  /// \brief Internal data class to hold individual flash light settings.
  /// A setting for each flash light is separately stored in a
  /// FlashLightSetting class, which takes care of dynamic specifications such
  /// as duration and interval.
  class FlashLightSetting
  {
    /// \brief Constructor.
    /// Initialize the setting by the data given to the base plugin.
    /// \param[in] _sdf SDF data for the setting.
    /// \param[in] _model The Model pointer holding the light to control.
    /// \param[in] _currentTime The current time point.
    public: FlashLightSetting(
      const sdf::ElementPtr &_sdf,
      const gazebo::physics::ModelPtr &_model,
      const gazebo::common::Time &_currentTime,
      gazebo_ros::Node::SharedPtr node);

    /// \brief Destructor.
    public: virtual ~FlashLightSetting();

    /// \brief Set the publisher and send an initial light command.
    /// \param[in] _pubLight The publisher to send a message
    public: virtual void InitPubLight(
      const gazebo::transport::PublisherPtr &_pubLight) final;

    /// \brief Update the light based on the given time.
    /// \param[in] _currentTime The current point of time to update the
    ///                         lights.
    public:
      virtual void UpdateLightInEnv(const gazebo::common::Time &_currentTime) final;

    /// \brief Getter of name.
    /// \return The name of the light element.
    public: virtual const std::string Name() const final;

    /// \brief Getter of link.
    /// \return A pointer to the link element.
    public: virtual const gazebo::physics::LinkPtr Link() const final;

    /// \brief Switch on (enable the flashlight).
    public: virtual void SwitchOn() final;

    /// \brief Switch off (disable the flashlight).
    public: virtual void SwitchOff() final;

    /// \brief Set the duration time for the specified block.
    /// \param[in] _duration New duration time to set.
    /// \param[in] _index The index to the block to update.
    public: virtual void SetDuration(
      const double _duration, const int _index) final;

    /// \brief Set the duration time for all the blocks.
    /// \param[in] _duration New duration time to set.
    public: virtual void SetDuration(const double _duration) final;

    /// \brief Set the interval time for the specified block.
    /// \param[in] _interval New interval time to set.
    /// \param[in] _index The index to the block to update.
    public: virtual void SetInterval(
      const double _interval, const int _index) final;

    /// \brief Set the interval time for all the blocks.
    /// \param[in] _interval New interval time to set.
    public: virtual void SetInterval(const double _interval) final;

    /// \brief Set the color for the specified block.
    /// \param[in] _color New color to set.
    /// \param[in] _index The index to the block to update.
    public: virtual void SetColor(
      const ignition::math::Color &_color, const int _index) final;

    /// \brief Set the color for all the blocks.
    /// \param[in] _color New color to set.
    public: virtual void SetColor(const ignition::math::Color &_color) final;

    /// \brief Get the number of blocks.
    /// \return The number of blocks the object currently has.
    public: virtual unsigned int BlockCount() final;

    /// \brief Remove a specified block.
    /// \param[in] _index Index to the block to remove
    /// \return True if the block exists and it was removed.
    public: virtual bool RemoveBlock(const int _index) final;

    /// \brief Insert a block.
    /// Create a block with specified parameters. If the index is out of range,
    /// the block will be appended at the end of the list.
    /// \param[in] _duration The duration for the block.
    /// \param[in] _interval The interval for the block.
    /// \param[in] _color The color for the block.
    /// \param[in] _index The index of the block to be inserted into the list.
    public: virtual void InsertBlock(
      const double _duration, const double _interval,
      const ignition::math::Color &_color, const int _index) final;

    /// \brief Flash the light
    /// This function is internally used to update the light in the
    /// environment.
    protected: virtual void Flash();

    /// \brief Dim the light
    /// This function is internally used to update the light in the
    /// environment.
    protected: virtual void Dim();

    /// \brief Get the current color of the light.
    /// This is to be used by an inheriting class of FlashLightSetting class.
    /// \return the color for the current block which the object is using.
    ///         It returns Black if there is no update about color.
    protected: virtual ignition::math::Color CurrentColor() final;

    /// \brief Pointer to private data
    private: std::unique_ptr<FlashLightSettingPrivate> dataPtr;

  };


  // forward declaration
  class FlashLightPluginPrivate;

  class FlashLightPlugin : public gazebo::ModelPlugin
  {
    /// \brief Constructor.
    public: FlashLightPlugin();

    /// \brief Destructor.
    public: virtual ~FlashLightPlugin();

    // Documentation inherited.
    public: void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;

    /// \brief Called by the world update start event
    protected: virtual void OnUpdate();

    /// \brief Turn on a flash light specified by the light name
    /// If more than one link have lights with the identical name,
    /// the first appearing light in the list will be updated.
    /// \param[in] _lightName The name of flash light
    /// \return True if the specified light is found.
    protected: virtual bool TurnOn(const std::string &_lightName) final;

    /// \brief Turn on a flash light specified by the name and its link
    /// \param[in] _lightName The name of flash light
    /// \param[in] _linkName The name of the link holding the light
    /// \return True if the specified light is found.
    protected: virtual bool TurnOn(
      const std::string &_lightName, const std::string &_linkName) final;

    /// \brief Turn on all flash lights
    /// \return True if there is one or more lights to turn on.
    protected: virtual bool TurnOnAll() final;

    /// \brief Turn off a flash light specified by the name
    /// If more than one link have lights with the identical name,
    /// the first appearing light in the list will be updated.
    /// \param[in] _lightName The name of flash light
    /// \return True if the specified light is found.
    protected: virtual bool TurnOff(const std::string &_lightName) final;

    /// \brief Turn off a flash light specified by the name
    /// \param[in] _lightName The name of flash light
    /// \param[in] _linkName The name of the link holding the light
    /// \return True if the specified light is found.
    protected: virtual bool TurnOff(
      const std::string &_lightName, const std::string &_linkName) final;

    /// \brief Turn off all flash lights
    /// \return True if there is one or more lights to turn off.
    protected: virtual bool TurnOffAll() final;

    /// \brief Change the duration of a specific block of the flashlight.
    /// If the index is a negative number, it updates all the blocks.
    /// \param[in] _lightName The name of flash light
    /// \param[in] _linkName The name of the link holding the light
    /// \param[in] _duration The new duration time to set
    /// \param[in] _index The index to the block to update
    /// \return True if the specified light is found.
    protected: virtual bool ChangeDuration(
      const std::string &_lightName, const std::string &_linkName,
      const double _duration, const int _index) final;

    /// \brief Change the duration of all the blocks of the flashlight.
    /// \param[in] _lightName The name of flash light
    /// \param[in] _linkName The name of the link holding the light
    /// \param[in] _duration The new duration time to set
    /// \return True if the specified light is found.
    protected: virtual bool ChangeDuration(
      const std::string &_lightName, const std::string &_linkName,
      const double _duration) final;

    /// \brief Change the interval of a specific block of the flashlight.
    /// If the index is a negative number, it updates all the blocks.
    /// \param[in] _lightName The name of flash light
    /// \param[in] _linkName The name of the link holding the light
    /// \param[in] _interval The new interval time to set
    /// \param[in] _index The index to the block to update
    /// \return True if the specified light is found.
    protected: virtual bool ChangeInterval(
      const std::string &_lightName, const std::string &_linkName,
      const double _interval, const int _index) final;

    /// \brief Change the interval of all the blocks of the flashlight.
    /// \param[in] _lightName The name of flash light
    /// \param[in] _linkName The name of the link holding the light
    /// \param[in] _interval The new interval time to set
    /// \return True if the specified light is found.
    protected: virtual bool ChangeInterval(
      const std::string &_lightName, const std::string &_linkName,
      const double _interval) final;

    /// \brief Change the color of a specific block of the flashlight.
    /// If the index is a negative number, it updates all the blocks.
    /// \param[in] _lightName The name of flash light
    /// \param[in] _linkName The name of the link holding the light
    /// \param[in] _color The new color to set
    /// \param[in] _index The index to the block to update
    /// \return True if the specified light is found.
    protected: virtual bool ChangeColor(
      const std::string &_lightName, const std::string &_linkName,
      const ignition::math::Color &_color, const int _index) final;

    /// \brief Change the color of all the blocks of the flashlight.
    /// \param[in] _lightName The name of flash light
    /// \param[in] _linkName The name of the link holding the light
    /// \param[in] _color The new color to set
    /// \return True if the specified light is found.
    protected: virtual bool ChangeColor(
      const std::string &_lightName, const std::string &_linkName,
      const ignition::math::Color &_color) final;

    /// \brief Create an object of setting.
    ///
    /// NOTE: This function is internally called in Load() of the base class.
    /// If a child class of FlashLightPlugin has also an inherited class of
    /// FlashLightSetting, this function must be overridden so that dataPtr
    /// deals with objects of the appropriate setting class.
    ///
    /// \param[in] _sdf SDF data for the setting.
    /// \param[in] _model The Model pointer holding the light to control.
    /// \param[in] _currentTime The current time point.
    /// \return A pointer to the newly created setting object.
    protected: virtual std::shared_ptr<FlashLightSetting> CreateSetting(
      const sdf::ElementPtr &_sdf,
      const gazebo::physics::ModelPtr &_model,
      const gazebo::common::Time &_currentTime,
    gazebo_ros::Node::SharedPtr node);

    /// \brief Initialize the additional part of an object of setting.
    ///
    /// NOTE: This function is internally called in Load() of the base class.
    /// If a child class of FlashLightPlugin has also an inherited class of
    /// FlashLightSetting, this function must be overridden so that the object
    /// can be initialized with necessary data. Also, the overridden function
    /// must call the original function in it.
    ///
    /// \param[in] _setting A pointer to the setting object.
    protected:
      virtual void InitSettingBySpecificData(
        std::shared_ptr<FlashLightSetting> &_setting);

    /// \brief Pointer to private data
    private: std::unique_ptr<FlashLightPluginPrivate> dataPtr;

        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr cmdledsubscription;

  };
}
#endif
