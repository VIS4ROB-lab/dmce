#include <ros/ros.h>

#include "dmce_test/common.hpp"

#include "dmce_core/TypeDefs.hpp"
#include "dmce_core/RobotMap.hpp"
#include "dmce_sim/OmplPathfinding.hpp"

namespace dmce_test {
	using namespace dmce;

	class TestOmplPathfinding : public testing::Test {
	public:
		RobotMap map_;
		OmplPathfinding pathfinder_;

		TestOmplPathfinding()
			: map_({20,20}, 1, {0,0}, "map"), pathfinder_(map_)
		{ }

	};

	TEST_F(TestOmplPathfinding, OnOutOfBounds_ThrowsError) {
		EXPECT_THROW(pathfinder_.makePath({-100,-100}, {0,0}), std::invalid_argument);
		EXPECT_THROW(pathfinder_.makePath({0,0}, {-100,-100}), std::invalid_argument);
	}

	TEST_F(TestOmplPathfinding, OnBlankMap_FindsPath) {
		pos_t from = {-5, 5};
		pos_t to = {5, 5};
		bool success; plan_t path;
		std::tie(success, path) = pathfinder_.makePath(from, to);
		ASSERT_TRUE(success);
		ASSERT_GE(path.size(), 2);
		EXPECT_FLOAT_EQ(path[0].pose.position.x, from.x());
		EXPECT_FLOAT_EQ(path[0].pose.position.y, from.y());
		EXPECT_FLOAT_EQ(path[path.size()-1].pose.position.x, to.x());
		EXPECT_FLOAT_EQ(path[path.size()-1].pose.position.y, to.y());
	}

	TEST_F(TestOmplPathfinding, OnObstacle_FindsNonCollidingPath) {
		pos_t from = {-5, 5};
		pos_t to = {5, 5};
		for (unsigned int i = 0; i < 10; i++)
			map_.setOccupancy(pos_t{0,i}, map_.occupiedValue);

		pathfinder_.updateMap(map_);
		bool success; plan_t path;
		std::tie(success, path) = pathfinder_.makePath(from, to);

		ASSERT_TRUE(success);
		ASSERT_GE(path.size(), 2);
		EXPECT_FLOAT_EQ(path[0].pose.position.x, from.x());
		EXPECT_FLOAT_EQ(path[0].pose.position.y, from.y());
		EXPECT_NEAR(path[path.size()-1].pose.position.x, to.x(), map_.getResolution());
		EXPECT_NEAR(path[path.size()-1].pose.position.y, to.y(), map_.getResolution());

		auto segmentCollides =
			[](const pos_t& from, const pos_t& to, const RobotMap& map) -> bool
		{
			for (auto it = map.getLineIterator(from, to); !it.isPastEnd(); ++it)
				if (map.isOccupied(*it))
					return true;
			return false;
		};

		for (unsigned int i = 1; i < path.size(); i++) {
			pos_t from = {path[i-1].pose.position.x, path[i-1].pose.position.y};
			pos_t to   = {path[i  ].pose.position.x, path[i  ].pose.position.y};
			ASSERT_FALSE(segmentCollides(from, to, map_));
		}
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "OmplPathfindingTestNode");
	testing::InitGoogleTest(&argc, argv);

	std::thread t([]{while(ros::ok()) ros::spin();});

	auto res = RUN_ALL_TESTS();
	
	ros::shutdown();
	return res;
}

