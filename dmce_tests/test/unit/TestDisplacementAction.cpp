#include "dmce_test/common.hpp"

#include "dmce_core/RobotMap.hpp"
#include "dmce_mcplanner/DisplacementAction.hpp"

namespace dmce_test {

	using namespace dmce;

	class TestDisplacementAction : public testing::Test {

	public:
		RobotMap map_;
		pos_t robotPos_;
		MCState state_;
		MCParams params_;

		double displacementAmount = DisplacementAction<NORTH>::getDisplacementAmount();

		TestDisplacementAction()
			: robotPos_(0,0), map_({50,50}, 1, {0,0}, "map"), state_{robotPos_, 1, map_}, params_(Funcs::getMCParams())
		{ }

		template<Displacement delta = Displacement::NORTH>
		DisplacementAction<delta> makeAction() {
			return DisplacementAction<delta>(state_, params_);
		}
	};

	TEST_F(TestDisplacementAction, OnBlankMap_isFeasible) {
		EXPECT_TRUE(makeAction<NORTH>().isFeasible(state_));
		EXPECT_TRUE(makeAction<SOUTH>().isFeasible(state_));
		EXPECT_TRUE(makeAction<EAST >().isFeasible(state_));
		EXPECT_TRUE(makeAction<WEST >().isFeasible(state_));
	}

	TEST_F(TestDisplacementAction, OnDestionationOccupied_isFeasible_returnsFalse) {
		state_.map.setOccupancy(dmce::pos_t(0, displacementAmount), 100);

		EXPECT_FALSE(makeAction<NORTH>().isFeasible(state_));

		EXPECT_TRUE(makeAction<SOUTH>().isFeasible(state_));
		EXPECT_TRUE(makeAction<EAST >().isFeasible(state_));
		EXPECT_TRUE(makeAction<WEST >().isFeasible(state_));
	}

	TEST_F(TestDisplacementAction, OnOutOfBounds_isNotFeasible) {
		double loc = map_.getLength().x()/2 - displacementAmount/2;
		state_.robot.pos = {-loc, -loc};
		EXPECT_FALSE(makeAction<SOUTH>().isFeasible(state_));
		EXPECT_FALSE(makeAction<WEST >().isFeasible(state_));

		state_.robot.pos = {loc, loc};
		EXPECT_FALSE(makeAction<NORTH>().isFeasible(state_));
		EXPECT_FALSE(makeAction<EAST >().isFeasible(state_));
	}

	TEST_F(TestDisplacementAction, OnCalcPosition_correct) {
		double expected = 0;

		auto test = [](MCAction& action, double expected) {
			pos_t initialPos = action.getInitialRobotState().pos;
			pos_t resultPos = action.getFinalRobotState().pos;
			EXPECT_FLOAT_EQ((resultPos - initialPos).norm(), expected);
		};

		auto north = makeAction<NORTH>();
		test(north, north.getDisplacementAmount());

		auto south = makeAction<SOUTH>();
		test(south, south.getDisplacementAmount());

		auto east = makeAction<EAST>();
		test(east, east.getDisplacementAmount());

		auto west = makeAction<WEST>();
		test(west, west.getDisplacementAmount());
	}
}

