#include "dmce_test/common.hpp"

#include "dmce_mcplanner/MCActionGenerator.hpp"

namespace dmce_test {

	using namespace dmce;

	class TestMCActionGenerator : public testing::Test {

	public:
		RobotMap map_;
		pos_t robotPos_;
		MCState state_;
		MCParams params_;
		MCAction* action_;
		MCActionGenerator generator_;
		size_t maxActionCount_;

		TestMCActionGenerator()
			: robotPos_(0, 0),
			  map_({50,50}, .1, {0,0}, "map"),
			  state_{robotPos_, 1, map_},
			  params_(Funcs::getMCParams()),
			  action_(new RandomDisplacement(state_, params_)),
			  generator_(MCActionGenerator()),
			  maxActionCount_(generator_.generator_map.size() + params_.randomDisplacementBranchingFactor + params_.frontierClusterBranchingFactor)
		{
			for (unsigned int i = 0; i < params_.frontierClusterBranchingFactor; ++i) {
				pos_t pos{0, i};
				map_.setOccupancy(pos, map_.freeValue);
			}
			state_.clusters.fromMapFrontiers(map_);
		}

		~TestMCActionGenerator() {
			delete action_;
		}
	};

	TEST_F(TestMCActionGenerator, OnBlankMap_generatesAllActions) {
		auto feasibleActions = generator_.generateFeasibleActions(state_, params_, nullptr, true, true);
		EXPECT_EQ(feasibleActions.size(), maxActionCount_);
	}

	TEST_F(TestMCActionGenerator, OnMapEdge_generatesFewerActions) {
		state_.robot.pos.x() = 24.5;
		state_.robot.pos.y() = 24.5;
		auto feasibleActions = generator_.generateFeasibleActions(state_, params_);
		EXPECT_LE(feasibleActions.size(), maxActionCount_);
	}
}

