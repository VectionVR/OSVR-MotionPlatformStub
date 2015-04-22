/** @file
	@brief Comprehensive example: Implementation of a dummy Hardware Detect
	Callback that creates a dummy device when it is "detected"

	@date 2014

	@author
	Sensics, Inc.
	<http://sensics.com/osvr>
	*/

// Copyright 2014 Sensics, Inc.
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

// Internal Includes
#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>

// Generated JSON header file
#include "com_vectionvr_osvr_motionPlatformDevicePlugin_json.h"
#include <boost/thread/thread.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>

// Library/third-party includes
// - none

// Standard includes
#include <iostream>

typedef boost::mt19937 RNGType;
typedef boost::uniform_int<int> DistributionType;

// Anonymous namespace to avoid symbol collision
namespace {

	class TrackerSyncDevice {
	public:
		TrackerSyncDevice(OSVR_PluginRegContext ctx) {
			/// Create the initialization options
			OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);
			// configure device tracker
			osvrDeviceTrackerConfigure(opts, &m_tracker);
			/// Create the sync device token with the options
			m_dev.initSync(ctx, "SyncMotionPlatformDevice", opts);
			/// Send JSON descriptor
			m_dev.sendJsonDescriptor(com_vectionvr_osvr_motionPlatformDevicePlugin_json);
			/// Register update callback
			m_dev.registerUpdateCallback(this);
		}

		OSVR_ReturnCode update() {
			/// initialise pose
			osvrPose3SetIdentity(&pose);
			/// update quaternion with random values
			updatePoseOrientation(getRandomFloat(-45.0f, 45.0f), getRandomFloat(-45.0f, 45.0f), getRandomFloat(-45.0f, 45.0f));
			/// send pose to listeners
			osvrDeviceTrackerSendPose(m_dev, m_tracker, &pose, 0);
#ifdef _DEBUG
			std::cout << "MPS_PLUGIN > Sending update" << pose.rotation << std::endl;
#endif
			boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
			return OSVR_RETURN_SUCCESS;
		}
	
	// random number generator properties
	private:
		RNGType rng;
	
	// OSVR related variables
	private:
		osvr::pluginkit::DeviceToken m_dev;
		OSVR_TrackerDeviceInterface m_tracker;
		OSVR_PoseState pose;
	
	// private methods
	private:
		int getRandomFloat(float min, float max){
			DistributionType u(min, max);
			boost::variate_generator<RNGType&, DistributionType > gen(rng, u);
			return gen();
		}
		/*
		 * Update pose with provided values
		 */
		void updatePoseOrientation(float pitch, float yaw, float roll)
		{
			// Basically we create 3 Quaternions, one for pitch, one for yaw, one for roll
			// and multiply those together.
			// the calculation below does the same, just shorter

			float p = pitch * (boost::math::constants::pi<float>() /180) / 2.0;
			float y = yaw * (boost::math::constants::pi<float>() / 180) / 2.0;
			float r = roll * (boost::math::constants::pi<float>() / 180) / 2.0;

			float sinp = sin(p);
			float siny = sin(y);
			float sinr = sin(r);
			float cosp = cos(p);
			float cosy = cos(y);
			float cosr = cos(r);

			osvrQuatSetX(&(pose.rotation), sinr * cosp * cosy - cosr * sinp * siny);
			osvrQuatSetY(&(pose.rotation), cosr * sinp * cosy + sinr * cosp * siny);
			osvrQuatSetZ(&(pose.rotation), cosr * cosp * siny - sinr * sinp * cosy);
			osvrQuatSetW(&(pose.rotation), cosr * cosp * cosy + sinr * sinp * siny);
		}
	};

	class HardwareDetection {
	public:
		HardwareDetection() : m_found(false) {}
		OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx) {
#ifdef _DEBUG
			std::cout << "MPS_PLUGIN > Got a hardware detection request" << std::endl;
#endif
			if (!m_found) {
				std::cout << "MPS_PLUGIN > We have detected our fake motion platform device - Starting setup !" << std::endl;
				m_found = true;
				/// Create our device object
				osvr::pluginkit::registerObjectForDeletion(ctx, new TrackerSyncDevice(ctx));
			}
			return OSVR_RETURN_SUCCESS;
		}

	private:
		/// @brief Have we found our device yet? (this limits the plugin to one
		/// instance)
		bool m_found;
	};
} // namespace

OSVR_PLUGIN(com_vectionvr_osvr_motionPlatformDevicePlugin) {
	osvr::pluginkit::PluginContext context(ctx);

	/// Register a detection callback function object.
	context.registerHardwareDetectCallback(new HardwareDetection());

	return OSVR_RETURN_SUCCESS;
}
