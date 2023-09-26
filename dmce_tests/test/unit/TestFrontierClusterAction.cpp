#include "dmce_test/common.hpp"
#include "dmce_core/RobotMap.hpp"
#include "dmce_mcplanner/FrontierClusterAction.hpp"

namespace dmce_test {

	using namespace dmce;
	using idx_t = grid_map::Index;

	class TestFrontierClusterAction : public testing::Test {

	public:
		RobotMap map_;
		FrontierClustering fc_;
		pos_t robotPos_;
		MCState state_;
		MCParams params_;
		std::vector<index_t> blockCenters = {{1,1},{9,8},{20,20}};

		TestFrontierClusterAction()
			: robotPos_(0,0),
			  map_({50,50}, 1, {0,0}, "map"),
			  params_(Funcs::getMCParams())
		{
			for (auto idx : blockCenters)
				free3x3Block(idx);
			fc_.fromMapFrontiers(map_);
		}

		void free3x3Block(index_t center) {
			// WARNING: does no bounds checking!
			for (int dx = -1; dx <= 1; dx++)
				for (int dy = -1; dy <= 1; dy++)
					map_.setOccupancy(idx_t{center.x()+dx,center.y()+dy}, map_.freeValue);
		}

		template<unsigned int clusterIndex>
		FrontierClusterAction<clusterIndex> makeAction() {
			fc_.fromMapFrontiers(map_);
			state_ = {robotPos_, 1, map_, fc_};
			return FrontierClusterAction<clusterIndex>(state_, params_);
		}
	};

	TEST_F(TestFrontierClusterAction, OnThreeClusters_index3_isNotFeasible) {
		ASSERT_EQ(blockCenters.size(), 3);
		EXPECT_TRUE(makeAction<0>().isFeasible(state_));
		EXPECT_TRUE(makeAction<1>().isFeasible(state_));
		EXPECT_TRUE(makeAction<2>().isFeasible(state_));
		EXPECT_FALSE(makeAction<3>().isFeasible(state_));
	}

	TEST_F(TestFrontierClusterAction, OnDestionationOccupied_isFeasible_returnsFalse) {
		auto sortedClusters = fc_.sortClustersByDistance(robotPos_);
		auto centroidPositions = fc_.getCentroidPositions();
		pos_t occPos = centroidPositions[sortedClusters[1].first];
		map_.setOccupancy(occPos, map_.occupiedValue);
		EXPECT_TRUE(makeAction<0>().isFeasible(state_));
		EXPECT_FALSE(makeAction<1>().isFeasible(state_));
		EXPECT_TRUE(makeAction<2>().isFeasible(state_));
	}

	TEST_F(TestFrontierClusterAction, CheckFinalPosition) {
		auto sortedClusters = fc_.sortClustersByDistance(robotPos_);
		auto centroidPositions = fc_.getCentroidPositions();
		pos_t expectedPos = centroidPositions[sortedClusters[1].first];
		auto action = makeAction<1>();
		auto resultPos = action.getFinalRobotState().pos;
		EXPECT_FLOAT_EQ((resultPos - expectedPos).norm(), 0);
		action.simulate(state_, 0);
		resultPos = state_.robot.pos;
		EXPECT_FLOAT_EQ((resultPos - expectedPos).norm(), 0);
	}
}

