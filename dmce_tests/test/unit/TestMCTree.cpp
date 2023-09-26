#include "dmce_test/common.hpp"

#include "dmce_mcplanner/MCTree.hpp"

namespace dmce_test {

	using namespace dmce;

	class TestMCTree : public testing::Test {
	public:
		RobotMap map_;
		pos_t robotPos_;
		MCState state_;
		MCParams params_;
		MCActionPtr root_action_;
		MCNodePtr root_node_;
		MCTree tree_;

		TestMCTree()
			: robotPos_(1,1),
			  map_({50,50}, 0.1, {0,0}, "map"),
			  state_{robotPos_, 1, map_},
			  params_(Funcs::getMCParams()),
			  root_action_(new DisplacementAction<Displacement::NONE>(state_, params_)),
			  root_node_(new MCTreeNode(state_, params_, root_action_)),
			  tree_(state_, root_node_, params_)
		{ }

		~TestMCTree() {

		}
	};

	TEST_F(TestMCTree, onIterate_treeSizeAndRootVisits_increaseByOne) {
		unsigned int initialSize = tree_.size();
		unsigned int initialVisits = tree_.getRoot()->getDiscountedVisits();
		EXPECT_EQ(initialSize, 1);
		for (uint i = 1; i < 30; ++i) {
			tree_.iterate();
			EXPECT_EQ(tree_.size(), initialSize+i);
			EXPECT_EQ(tree_.getRoot()->getDiscountedVisits(), initialVisits+i);
		}
	}

	TEST_F(TestMCTree, onRootOnly_getPlan_sizeZero) {
		auto plan = tree_.getCurrentBestPlan();
		EXPECT_EQ(plan.size(), 0);
	}

	TEST_F(TestMCTree, onIterate_planSize_increases) {
		unsigned int initialSize = tree_.getCurrentBestPlan().size();
		while (tree_.getRoot()->hasPotentialChildren()) {
			tree_.iterate();
		}
		EXPECT_GT(tree_.getCurrentBestPlan().size(), initialSize);
	}

	TEST_F(TestMCTree, onGetPlan_eachElement_isChildOfPrevious) {
		state_.robot.pos += pos_t{2, 2};
		tree_.updateState(state_);
		while(tree_.getCurrentBestPlan().size() < 3) {
			tree_.iterate();
		}
		auto plan = tree_.getCurrentBestPlan();
		auto it = ++plan.begin();
		auto prevIt = plan.begin();
		for ( ; it != plan.end(); ++it, ++prevIt) {
			auto curEl = *it;
			auto prevEl = *prevIt;
			EXPECT_EQ(curEl->parentNode, prevEl);
		}
	}
}
