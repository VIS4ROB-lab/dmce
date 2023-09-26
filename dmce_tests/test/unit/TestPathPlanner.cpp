#include "dmce_test/common.hpp"

#include "dmce_sim/PathPlanner.hpp"
#include "dmce_core/RobotMap.hpp"

namespace dmce_test {

	class DummyPathPlanner : public dmce::PathPlanner {
	public:
		using dmce::PathPlanner::PathPlanner;

		bool move(const double& timeStep, dmce::pos_t& position) override {
			auto target = getTarget();
			callCount++;
			return false;
		}
		unsigned int callCount = 0;
	};


	class TestPathPlanner : public testing::Test {
	protected:
		DummyPathPlanner* planner_ = nullptr;

	public:
		TestPathPlanner() {
			dmce::RobotMap map;
			if (planner_ == nullptr) {
				planner_ = new DummyPathPlanner();
				planner_->updateMap(map);
			}
		}

		~TestPathPlanner() {
			if (planner_ != nullptr)
				delete planner_;
		}
	};

	TEST_F(TestPathPlanner, OnSetTarget_GetTarget_ReturnsCorrect) {
		grid_map::Position target{10, 11};
		planner_->setTarget(target);
		EXPECT_EQ(planner_->getTarget(), target);
	}

	TEST_F(TestPathPlanner, OnNoSetTarget_GetTarget_throwsError) {
		EXPECT_THROW(planner_->getTarget(), std::runtime_error);
	}

	TEST_F(TestPathPlanner, OnNoSetTarget_move_throwsError) {
		grid_map::Position pos{0,0};
		EXPECT_THROW(planner_->move(1, pos), std::runtime_error);
	}

	TEST_F(TestPathPlanner, OnNoSetTarget_makePlan_throwsError) {
		grid_map::Position robotPos{0,0};
		EXPECT_THROW(planner_->makePlan(robotPos), std::runtime_error);
	}

	TEST_F(TestPathPlanner, OnNoMakePlan_getPlan_throwsError) {
		grid_map::Position target{10, 11};
		grid_map::Position robotPos{0,0};
		planner_->setTarget(target);
		EXPECT_THROW(planner_->getPlan(), std::runtime_error);
	}

	TEST_F(TestPathPlanner, OnMove_CallCount_Increases) {
		planner_->setTarget({10, 11});
		unsigned int initCallCount = planner_->callCount;
		grid_map::Position pos{0,0};
		planner_->move(1, pos);
		EXPECT_EQ(planner_->callCount, initCallCount+1);
	}
}
