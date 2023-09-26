#include "mre_dmcts_test/common.hpp"
#include <ros/package.h>

#include "mre_dmcts_sim/GroundTruthMap.hpp"
#include "mre_dmcts_sim/CollisionHandler.hpp"

namespace mre_dmcts_test {

	TEST(TestCollisionHandler, OnCollidingObstacles_collides_returnsTrue) {
		std::string packagePath = ros::package::getPath("mre_dmcts_sim") + "/";
		double resolution = 1;
		mre_dmcts::GroundTruthMap map(
			packagePath + "maps/testMap_10x10.png",
			resolution
		);
		double diameter = 2;
		mre_dmcts::CollisionHandler ch(map, diameter, resolution);
		grid_map::Position robotPos{0,0.1};

		EXPECT_TRUE(ch.collides(robotPos));
		robotPos = {3, -3};
		EXPECT_FALSE(ch.collides(robotPos));
	}

	TEST(TestCollisionHandler, OnCollidingObstacles_handleCollsions_resolves) {
		std::string packagePath = ros::package::getPath("mre_dmcts_sim") + "/";
		double resolution = 1;
		mre_dmcts::GroundTruthMap map(
			packagePath + "maps/testMap_10x10.png",
			resolution
		);
		double diameter = 2;
		mre_dmcts::CollisionHandler ch(map, diameter, resolution);
		grid_map::Position robotPos{0,0.1};

		auto distToClosestObstacle =
		    [&map,&diameter](grid_map::Position robotPos) -> double {
				double closest = std::numeric_limits<double>::infinity();
				auto it = map.getCircleIterator(robotPos, diameter/2);
				for (; !it.isPastEnd(); ++it) {
					if (map.isOccupied(*it)) {
						grid_map::Position pos;
						map.getPosition(*it, pos);
						double dist = (robotPos - pos).norm();
						closest = std::min(closest, dist);
					}
				}
				return closest;
			};

		EXPECT_LT(distToClosestObstacle(robotPos), diameter/2);

		EXPECT_TRUE(ch.handleCollisions(robotPos));

		EXPECT_GT(distToClosestObstacle(robotPos), diameter/2);
	}
}
