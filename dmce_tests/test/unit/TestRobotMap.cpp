#include "dmce_test/common.hpp"

#include "dmce_core/RobotMap.hpp"

namespace dmce_test {
class TestRobotMap : public ::testing::Test {
protected:
	const grid_map::Length length{4.5, 8};
	const grid_map::Position position{1, 2};
	const double resolution = 0.05;
	const std::string frameId = "map";
	const double initialValue = 60;

	dmce::RobotMap robotMap;
	dmce::RobotMap otherMap;

public:
	TestRobotMap()
		: robotMap(length, resolution, position, frameId, initialValue),
		  otherMap(length, resolution, {0,0}, frameId, initialValue)
	{

	}
};

TEST_F(TestRobotMap, SetSingleOccupancyValue) {
	grid_map::Position p(2, 3);

	robotMap.setOccupancy(p, 100);
	EXPECT_FLOAT_EQ(robotMap.getOccupancy(p), 100);

	unsigned int nCells = robotMap.getNCells();
	EXPECT_DOUBLE_EQ(robotMap.getRelativeEntropy(), (nCells-1.)/nCells);

	robotMap.setOccupancy(p, 50);
	EXPECT_DOUBLE_EQ(robotMap.getRelativeEntropy(), 1.0);
}

TEST_F(TestRobotMap, OnInvalidPositions_ThrowsNoError) {
	using vd = std::vector<double>;
	using vf = std::vector<float>;

	EXPECT_NO_THROW(robotMap.setOccupancy(vd{3.3}, vd{0}, vf{0},  1));
	EXPECT_NO_THROW(robotMap.setOccupancy(vd{-1.3}, vd{0},vf{0},  1));
	EXPECT_NO_THROW(robotMap.setOccupancy(vd{0}, vd{6.1}, vf{0},  1));
	EXPECT_NO_THROW(robotMap.setOccupancy(vd{0}, vd{-2.1},vf{0},  1));
}

TEST_F(TestRobotMap, OnInvalidValues_ThrowsError) {
	using vd = std::vector<double>;
	using vf = std::vector<float>;

	EXPECT_THROW(robotMap.setOccupancy(vd{0}, vd{0},   vf{101},1), std::invalid_argument);
	EXPECT_THROW(robotMap.setOccupancy(vd{0}, vd{0},   vf{-.1},1), std::invalid_argument);
}

TEST_F(TestRobotMap, SetMultipleOccupancyValues) {
	uint n = 4;
	std::vector<double> x = {0, 1, 2, 3};
	std::vector<double> y = {0, 1, 2, 3};
	std::vector<float>  v = {0, 10, 90, 100};

	robotMap.setOccupancy(x, y, v, n);
	for (uint i = 0; i < n; ++i) {
		grid_map::Position pos{x[i], y[i]};
		EXPECT_FLOAT_EQ(robotMap.getOccupancy(pos), v[i]);
	}
}

TEST_F(TestRobotMap, IsFrontierCell) {
	using idx_t = grid_map::Index;
	idx_t idx{2, 2};
	EXPECT_FALSE(robotMap.isFrontierCell(idx));
	
	robotMap.setOccupancy(idx, robotMap.freeValue);
	EXPECT_TRUE(robotMap.isFrontierCell(idx));
	dmce::pos_t pos; robotMap.getPosition(idx, pos);
	EXPECT_TRUE(robotMap.isFrontierCell(pos));

	for (unsigned int x = 1; x <= 3; x++) {
		for (unsigned int y = 1; y <= 3; y++) {
			robotMap.setOccupancy(idx_t{x,y}, robotMap.occupiedValue);
		}
	}
	EXPECT_FALSE(robotMap.isFrontierCell(idx));
	robotMap.setOccupancy(idx, robotMap.freeValue);
	EXPECT_FALSE(robotMap.isFrontierCell(idx));
}

TEST_F(TestRobotMap, OnOneFreeCell_GetFrontier_SizeOne) {
	dmce::indexList_t frontier = robotMap.getFrontier();
	EXPECT_EQ(frontier.size(), 0);

	grid_map::Index idx{2,2};
	robotMap.setOccupancy(idx, 0);
	frontier = robotMap.getFrontier();
	EXPECT_EQ(frontier.size(), 1);
}

TEST_F(TestRobotMap, OnNineFreeCells_GetFrontier_SizeEight) {
	dmce::indexList_t frontier = robotMap.getFrontier();
	EXPECT_EQ(frontier.size(), 0);

	for (unsigned int x = 1; x <= 3; x++) {
		for (unsigned int y = 1; y <= 3; y++) {
			robotMap.setOccupancy(grid_map::Index{x,y}, robotMap.freeValue);
		}
	}
	frontier = robotMap.getFrontier();
	EXPECT_EQ(frontier.size(), 8);
}

TEST_F(TestRobotMap, onFree_setOccupancyConservative) {
	grid_map::Position p{0,0};
	robotMap.setOccupancyConservative(p, robotMap.freeValue);
	EXPECT_TRUE(robotMap.isFree(p));

	robotMap.setOccupancyConservative(p, robotMap.unknownValue);
	EXPECT_TRUE(robotMap.isKnown(p));

	robotMap.setOccupancyConservative(p, robotMap.occupiedValue);
	EXPECT_TRUE(robotMap.isOccupied(p));
}

TEST_F(TestRobotMap, onOccupied_setOccupancyConservative) {
	grid_map::Position p{1,0};
	robotMap.setOccupancyConservative(p, robotMap.occupiedValue);
	EXPECT_TRUE(robotMap.isOccupied(p));

	robotMap.setOccupancyConservative(p, robotMap.unknownValue);
	EXPECT_TRUE(robotMap.isKnown(p));

	robotMap.setOccupancyConservative(p, robotMap.freeValue);
	EXPECT_FALSE(robotMap.isFree(p));
}

TEST_F(TestRobotMap, mergeMap_unknownPrior) {
	// If prior is unknown, inherit value from other map
	dmce::pos_t occPos{.5,0};
	dmce::pos_t freePos{.5,1};
	otherMap.setOccupancy(occPos, otherMap.occupiedValue);
	otherMap.setOccupancy(freePos, otherMap.freeValue);

	robotMap.merge(otherMap);
	EXPECT_TRUE(robotMap.isOccupied(occPos));
	EXPECT_TRUE(robotMap.isFree(freePos));
}

TEST_F(TestRobotMap, mergeMap_freePrior) {
	// If prior is free, inherit obstacles only
	dmce::pos_t occPos = {0,1};
	dmce::pos_t freePos = {0,.5};
	robotMap.setOccupancy(occPos, robotMap.freeValue);
	robotMap.setOccupancy(freePos, robotMap.freeValue);
	otherMap.setOccupancy(occPos, otherMap.occupiedValue);
	otherMap.setOccupancy(freePos, otherMap.unknownValue);

	robotMap.merge(otherMap);
	EXPECT_TRUE(robotMap.isOccupied(occPos));
	EXPECT_TRUE(robotMap.isFree(freePos));
}

TEST_F(TestRobotMap, mergeMap_occupiedPrior) {
	// If prior is occupied, don't change it
	dmce::pos_t unknPos = {1,.5};
	dmce::pos_t freePos = {1,0};
	robotMap.setOccupancy(unknPos, robotMap.occupiedValue);
	robotMap.setOccupancy(freePos, robotMap.occupiedValue);
	otherMap.setOccupancy(unknPos, otherMap.unknownValue);
	otherMap.setOccupancy(freePos, otherMap.freeValue);

	robotMap.merge(otherMap);
	EXPECT_TRUE(robotMap.isOccupied(unknPos));
	EXPECT_TRUE(robotMap.isOccupied(freePos));
}
}
