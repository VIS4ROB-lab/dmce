#include "mre_dmcts_test/common.hpp"

#include "mre_dmcts_core/RandomPlanner.hpp"
#include "mre_dmcts_core/FrontierPlanner.hpp"
#include "mre_dmcts_mcplanner/MCTSPlanner.hpp"
#include "mre_dmcts_mcplanner/DMCTSPlanner.hpp"

namespace mre_dmcts_test {

	template<typename T>
	class TestPlanner : public testing::Test {

	public:
		mre_dmcts::RobotMap map_;
		mre_dmcts::pos_t position_;
		T* plannerInstance_;
		mre_dmcts::MCParams params_;

		TestPlanner()
			: position_(5,7), params_(Funcs::getMCParams())
		{
			plannerInstance_ = new T(1.0);
			plannerInstance_->setMap(map_);
		}
	};

	using PlannerTypes = ::testing::Types<mre_dmcts::RandomPlanner, mre_dmcts::FrontierPlanner, mre_dmcts::MCTSPlanner>;
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
