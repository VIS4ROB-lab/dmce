#include "dmce_test/common.hpp"
#include <ros/package.h>

#include "dmce_sim/Navigation.hpp"
#include "dmce_core/RobotMap.hpp"

namespace dmce_test {

	using namespace dmce;


	class TestNavigation : public testing::Test {
	protected:
		Navigation* nav_;
		std::string packagePath_;
		pos_t initialPosition_{0, 0};
		pos_t intermediatePosition_{5, 5};
		pos_t goalPosition_{10, 0};
		pos_t invalidGoal_{-15, 0};
		pos_t outOfBoundsPosition_{-500, 0};
		double robotSpeed_ = 1;
		ros::Duration timeStep_{1};
		RobotMap map_;

		plan_t plan_;
		plan_t simplePath_;
		plan_t longPath_;
		plan_t invalidGoalPath_;
		plan_t invalidGoalPlan_;

	public:

		TestNavigation()
		 : packagePath_(ros::package::getPath("dmce_sim") + "/"),
		   map_({312.5,312.5}, .5, {0,0}, "map")
		{
			GroundTruthMap truthMap{
				packagePath_ + "maps/testMap_binary.png",
				0.5,
				"map"
			};

			if (truthMap.isOccupied(initialPosition_))
				throw std::runtime_error("Initial position infeasible!");

			if (truthMap.isOccupied(intermediatePosition_))
				throw std::runtime_error("Intermediate position infeasible!");

			if (truthMap.isOccupied(goalPosition_))
				throw std::runtime_error("Goal position infeasible!");

			if (!truthMap.isOccupied(invalidGoal_))
				throw std::runtime_error("Invalid goal position is valid!");

			if (truthMap.isInside(outOfBoundsPosition_))
				throw std::runtime_error("Out-of-bounds position is inside the map!");

			nav_ = new Navigation(truthMap, initialPosition_, robotSpeed_);

			plan_.push_back(posToPose(goalPosition_));

			simplePath_.push_back(posToPose(initialPosition_));
			simplePath_.push_back(posToPose(goalPosition_));

			longPath_.push_back(posToPose(initialPosition_));
			longPath_.push_back(posToPose(intermediatePosition_));
			longPath_.push_back(posToPose(goalPosition_));

			invalidGoalPath_.push_back(posToPose(initialPosition_));
			invalidGoalPath_.push_back(posToPose(invalidGoal_));

			invalidGoalPlan_.push_back(posToPose(invalidGoal_));
		}

		void initNav() {
			nav_->updateNavigationMap(map_);
			nav_->updatePlan(plan_);
			nav_->updatePath(simplePath_);
		}

		~TestNavigation() {
			delete nav_;
		}

	};

	TEST_F(TestNavigation, OnGoalOutOfBounds_updatePlan_rejectsPlan) {
		initNav();
		plan_t outOfBoundsPlan;
		outOfBoundsPlan.push_back(posToPose(outOfBoundsPosition_));
		nav_->updatePlan(outOfBoundsPlan);
		EXPECT_FALSE(nav_->hasGoal());
	}


	TEST_F(TestNavigation, OnNoNavMap_updatePlan_RejectsPlan) {
		nav_->updatePlan(plan_);
		EXPECT_FALSE(nav_->hasGoal());
	}

	TEST_F(TestNavigation, OnNoPath_move_DoesNothing) {
		nav_->updateNavigationMap(map_);
		nav_->updatePlan(plan_);
		nav_->moveRobotToGoal(timeStep_);
		pos_t finalPos = nav_->getRobotPosition();
		EXPECT_FLOAT_EQ(finalPos.x(), initialPosition_.x());
		EXPECT_FLOAT_EQ(finalPos.y(), initialPosition_.y());
	}

	TEST_F(TestNavigation, OnNoPlan_move_DoesNothing) {
		nav_->updatePath(simplePath_);
		nav_->updateNavigationMap(map_);

		EXPECT_FALSE(nav_->hasGoal());
		nav_->moveRobotToGoal(timeStep_);
		EXPECT_FALSE(nav_->hasGoal());
		pos_t finalPos = nav_->getRobotPosition();
		EXPECT_FLOAT_EQ(finalPos.x(), initialPosition_.x());
		EXPECT_FLOAT_EQ(finalPos.y(), initialPosition_.y());
	}

	TEST_F(TestNavigation, OnHasMapAndPlanAndPath_move_IsCorrect) {
		initNav();
		pos_t direction = (goalPosition_ - initialPosition_).normalized();
		pos_t expectedPosition = timeStep_.toSec() * robotSpeed_ * direction;

		nav_->moveRobotToGoal(timeStep_);
		pos_t finalPos = nav_->getRobotPosition();
		EXPECT_FLOAT_EQ(finalPos.x(), expectedPosition.x());
		EXPECT_FLOAT_EQ(finalPos.y(), expectedPosition.y());
	}

	TEST_F(TestNavigation, OnOvershoot_move_StopsAtGoal) {
		initNav();
		pos_t expectedPosition = goalPosition_;

		nav_->moveRobotToGoal(ros::Duration(100));
		pos_t finalPos = nav_->getRobotPosition();
		EXPECT_FLOAT_EQ(finalPos.x(), expectedPosition.x());
		EXPECT_FLOAT_EQ(finalPos.y(), expectedPosition.y());
	}

	TEST_F(TestNavigation, OnNoPlan_hasGoal_IsCorrect) {
		EXPECT_FALSE(nav_->hasGoal());
	}

	TEST_F(TestNavigation, OnNoPlan_getCurrentGoal_ThrowsError) {
		EXPECT_THROW(nav_->getCurrentGoal(), std::runtime_error);
	}

	TEST_F(TestNavigation, OnPlanSet_getCurrentGoal_ReturnsFirst) {
		initNav();
		pos_t goal = nav_->getCurrentGoal();
		EXPECT_FLOAT_EQ(goal.x(), goalPosition_.x());
		EXPECT_FLOAT_EQ(goal.y(), goalPosition_.y());
	}

	TEST_F(TestNavigation, OnGoalReached_move_requestsNewPlan) {
		initNav();
		pos_t expectedPosition = goalPosition_;

		nav_->moveRobotToGoal(timeStep_);
		EXPECT_TRUE(nav_->hasGoal());

		nav_->moveRobotToGoal(ros::Duration(100));
		pos_t finalPos = nav_->getRobotPosition();
		EXPECT_FLOAT_EQ(finalPos.x(), expectedPosition.x());
		EXPECT_FLOAT_EQ(finalPos.y(), expectedPosition.y());
		EXPECT_TRUE(nav_->wasLastPlanSuccessful());
		EXPECT_FALSE(nav_->hasGoal());
	}

	TEST_F(TestNavigation, OnGoalOccupied_move_abortsPlan) {
		initNav();
		map_.setOccupancy(goalPosition_, map_.occupiedValue);
		nav_->updateNavigationMap(map_);
		nav_->moveRobotToGoal(timeStep_);
		pos_t finalPos = nav_->getRobotPosition();
		EXPECT_FLOAT_EQ(finalPos.x(), initialPosition_.x());
		EXPECT_FLOAT_EQ(finalPos.y(), initialPosition_.y());
		EXPECT_FALSE(nav_->hasGoal());
		EXPECT_FALSE(nav_->wasLastPlanSuccessful());
	}

	TEST_F(TestNavigation, onObstacleAppears_hasValidPath_rejectsPath) {
		EXPECT_FALSE(nav_->hasValidPath());
		initNav();
		EXPECT_TRUE(nav_->hasValidPath());
		pos_t middle = initialPosition_ + .5 * (goalPosition_-initialPosition_);
		map_.setOccupancy(middle, map_.occupiedValue);
		nav_->updateNavigationMap(map_);
		EXPECT_FALSE(nav_->hasValidPath());
	}

	TEST_F(TestNavigation, OnPathNotToGoal_updatePath_rejectsPath) {
		nav_->updateNavigationMap(map_);
		nav_->updatePlan(plan_);
		plan_t notActualPath;
		notActualPath.push_back(posToPose(initialPosition_));
		notActualPath.push_back(posToPose(intermediatePosition_));
		nav_->updatePath(notActualPath);

		EXPECT_FALSE(nav_->hasValidPath());
	}

	TEST_F(TestNavigation, OnLongerPath_updatePath_rejectsPath) {
		initNav();
		double initLen = nav_->getCurrentPathLength();
		nav_->updatePath(longPath_);
		EXPECT_DOUBLE_EQ(initLen, nav_->getCurrentPathLength());
	}

	TEST_F(TestNavigation, OnShorterPath_updatePath_acceptsPath) {
		nav_->updateNavigationMap(map_);
		nav_->updatePlan(plan_);
		nav_->updatePath(longPath_);
		EXPECT_TRUE(nav_->hasValidPath());
		double initLen = nav_->getCurrentPathLength();
		nav_->updatePath(simplePath_);
		EXPECT_LT(nav_->getCurrentPathLength(), initLen);
	}

	TEST_F(TestNavigation, OnObstacleAppears_updatePath_acceptsLongerPath) {
		initNav();
		double initLen = nav_->getCurrentPathLength();
		pos_t middle = initialPosition_ + .5 * (goalPosition_-initialPosition_);
		map_.setOccupancy(middle, map_.occupiedValue);
		nav_->updateNavigationMap(map_);

		nav_->updatePath(longPath_);
		EXPECT_TRUE(nav_->hasValidPath());
		EXPECT_GT(nav_->getCurrentPathLength(), initLen);
	}

	TEST_F(TestNavigation, OnObstacleAppears_move_DoesNothing) {
		initNav();
		double initLen = nav_->getCurrentPathLength();
		pos_t middle = initialPosition_ + .5 * (goalPosition_-initialPosition_);
		map_.setOccupancy(middle, map_.occupiedValue);
		nav_->updateNavigationMap(map_);

		nav_->moveRobotToGoal(timeStep_);
		EXPECT_TRUE(nav_->hasGoal());
		pos_t finalPos = nav_->getRobotPosition();
		EXPECT_FLOAT_EQ(finalPos.x(), initialPosition_.x());
		EXPECT_FLOAT_EQ(finalPos.y(), initialPosition_.y());
	}

	TEST_F(TestNavigation, OnChangeGoal_hasValidPath_RejectsPath) {
		initNav();
		nav_->updatePlan(invalidGoalPlan_);
		EXPECT_FALSE(nav_->hasValidPath());
	}

	TEST_F(TestNavigation, OnChangeGoal_updatePath_acceptsLongerPath) {
		initNav();
		nav_->updatePlan(invalidGoalPlan_);
		EXPECT_FALSE(nav_->hasValidPath());

		nav_->updatePath(invalidGoalPath_);
		EXPECT_TRUE(nav_->hasValidPath());
	}

	TEST_F(TestNavigation, OnCollidingPath_move_detectsCollision) {
		initNav();
		nav_->updatePlan(invalidGoalPlan_);
		nav_->updatePath(invalidGoalPath_);
		EXPECT_TRUE(nav_->hasValidPath());
		nav_->moveRobotToGoal(timeStep_);

		EXPECT_FALSE(nav_->hasGoal());
		pos_t finalPos = nav_->getRobotPosition();
		EXPECT_FLOAT_EQ(finalPos.x(), initialPosition_.x());
		EXPECT_FLOAT_EQ(finalPos.y(), initialPosition_.y());
	}
}

