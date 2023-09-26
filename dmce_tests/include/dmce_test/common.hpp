#pragma once

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>

#include "dmce_core/OccupancyMap.hpp"
#include "dmce_mcplanner/MCParams.hpp"

namespace dmce_test {
	/**
	 * Temporarily remove a given parameter from the ROS parameter server.
	 * The parameter is reinstated when *this goes out of scope.
	 * Useful to test how things behave when a parameter they need is missing.
	 */
	template<typename T>
	class StealParameter {
		T stolen_value;
		std::string param_name;
	public:
		StealParameter(const std::string& name) : param_name(name) {
			ros::param::get(name, stolen_value);
			ros::param::del(name);
		}
	
		~StealParameter() {
			// Reinstate the parameter, since the parameter server is persistent
			ros::param::set(param_name, stolen_value);
		}
	};

	struct Funcs {
		/**
		 * Suspend the current thread for a short time.
		 * Useful to wait for nodes to start.
		 */
		static void takeANap(unsigned int ms = 10) {
			std::this_thread::sleep_for(std::chrono::milliseconds(ms));
		}

		/**
		 * Check a given map to make sure it matches the test map (maps/testMap.png)
		 */
		static void checkTestMap(
			const dmce::OccupancyMap& map,
			const double& resolution = 1)
		{
			EXPECT_EQ(map.getFrameId(), "map");
			auto l = map.getLength();
			EXPECT_FLOAT_EQ(l.x(), 3*resolution);
			EXPECT_FLOAT_EQ(l.y(), 1*resolution);

			auto it = map.getGridMapIterator();
			EXPECT_FLOAT_EQ(map.getOccupancy(*it), 0); // Black
			EXPECT_FLOAT_EQ(map.getOccupancy(*(++it)), 50); // Mid-grey
			EXPECT_FLOAT_EQ(map.getOccupancy(*(++it)), 100); // White
		}

		/**
		 * Check that the given function throws an std::runtime_error whenever
		 * any of the given parameters is missing from the ROS parameter server.
		 * @param params List of parameter names that are required by the function
		 * @param f The callable to be executed, which is supposed to throw errors.
		 */
		template<typename F>
		static void requireParams(std::vector<std::string> params, F f) {
			for (unsigned int i = 0; i < params.size(); i++) {
				StealParameter<double> stolen(params[i]);
				ASSERT_THROW(f(), std::runtime_error);
			}
		}

		template<typename T>
		static void expectVectorEqual(const T& v1, const T& v2) {
			EXPECT_DOUBLE_EQ((v1 - v2).norm(), 0.0);
		}

		static dmce::MCParams getMCParams() {
			dmce::MCParams params;
			params.reuseBranches = true;
			params.rolloutDepth = 5;
			params.minRollouts = 10;
			params.minPlanDepth = 1;
			params.minPlanValue = .001;
			params.explorationFactor = .5;
			params.iterationDiscountFactor = 1;
			params.robotLidarRayCount = 45;
			params.robotSensorRange = 10;
			params.robotSpeed = 1;
			params.useActionCaching = false;
			params.useLocalReward = true;
			params.spatialHashBinSize = 1;
			params.navigationCutoff = 0.1;
			params.randomDisplacementMaxTurnAngle = 90;
			params.randomDisplacementMinSpread = 0.1;
			params.randomDisplacementLength = 1;
			params.randomDisplacementBranchingFactor = 4;
			params.planBufferSize = 5;
			params.timeDiscountFactor = 1;
			params.actionBaseDuration = 0;
			params.frontierClusterBranchingFactor = 5;
			return params;
		}
	};
	
};
