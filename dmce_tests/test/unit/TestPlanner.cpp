#include "dmce_test/common.hpp"

#include "dmce_core/RandomPlanner.hpp"
#include "dmce_core/FrontierPlanner.hpp"
#include "dmce_mcplanner/MCTSPlanner.hpp"
#include "dmce_mcplanner/DMCTSPlanner.hpp"

namespace dmce_test {

	template<typename T>
	class TestPlanner : public testing::Test {

	public:
		dmce::RobotMap map_;
		dmce::pos_t position_;
		T* plannerInstance_;
		dmce::MCParams params_;

		TestPlanner()
			: position_(5,7), params_(Funcs::getMCParams())
		{
			plannerInstance_ = new T(1.0);
			plannerInstance_->setMap(map_);
		}
	};

	using PlannerTypes = ::testing::Types<dmce::RandomPlanner, dmce::FrontierPlanner, dmce::MCTSPlanner>;
	TYPED_TEST_SUITE(TestPlanner, PlannerTypes);

	TYPED_TEST(TestPlanner, OnSetPosition_Position_MatchesInput) {
		this->plannerInstance_->setPosition(this->position_);
		EXPECT_EQ(this->plannerInstance_->getPosition(), this->position_);
	}

	TYPED_TEST(TestPlanner, OnNoUpdatePlan_getPlan_returnsEmptyPlan) {
		this->plannerInstance_->setPosition(this->position_);
		auto plan = this->plannerInstance_->getLatestPlan();
		EXPECT_EQ(plan.second.size(), 0);
	}
}
