#include "dmce_test/common.hpp"

#include "dmce_core/OccupancyMap.hpp"
#include "dmce_core/RobotMap.hpp"

namespace dmce_test {
class TestOccupancyMap : public ::testing::Test {
protected:
	const grid_map::Length length{4.5, 8};
	const grid_map::Position position{1, 2};
	const double resolution = 0.05;
	const std::string frameId = "map";
	const double initialValue = 60;

	dmce::OccupancyMap occMap_;

public:
	TestOccupancyMap() : occMap_(length, resolution, position, frameId, initialValue) {

	}

	void checkParams(dmce::OccupancyMap occMap) {
		auto p = occMap.getPosition();
		EXPECT_EQ(p, position);
		EXPECT_EQ(occMap.getResolution(), resolution);
		auto l = occMap.getLength();
		EXPECT_NEAR(l.x(), length.x(), resolution);
		EXPECT_NEAR(l.y(), length.y(), resolution);
		EXPECT_EQ(occMap.getFrameId(), frameId);
	}
};

TEST_F(TestOccupancyMap, CheckParameters) {
	checkParams(occMap_);
}

TEST_F(TestOccupancyMap, CheckInitialValue) {
	for (auto it = occMap_.getGridMapIterator(); !it.isPastEnd(); ++it) {
		EXPECT_FLOAT_EQ(occMap_.getOccupancy(*it), initialValue);
	}
}

TEST_F(TestOccupancyMap, ConstructFromMessage) {
	grid_map_msgs::GridMap msg = occMap_.getMessage();
	dmce::OccupancyMap newOccMap(msg);
	checkParams(newOccMap);
}

TEST_F(TestOccupancyMap, IsOccupied) {
	auto it = occMap_.getGridMapIterator();
	EXPECT_FALSE(occMap_.isOccupied(*it));

	dmce::OccupancyMap map(length, resolution, position, frameId, 98);
	EXPECT_TRUE(map.isOccupied(*it));

	map = dmce::OccupancyMap(length, resolution, position, frameId, 3);
	EXPECT_FALSE(map.isOccupied(*it));
}

TEST_F(TestOccupancyMap, IsFree) {
	auto it = occMap_.getGridMapIterator();
	EXPECT_FALSE(occMap_.isFree(*it));

	dmce::OccupancyMap map(length, resolution, position, frameId, 98);
	EXPECT_FALSE(map.isFree(*it));

	map = dmce::OccupancyMap(length, resolution, position, frameId, 03);
	EXPECT_TRUE(map.isFree(*it));
}

TEST_F(TestOccupancyMap, IsKnown) {
	auto it = occMap_.getGridMapIterator();
	EXPECT_FALSE(occMap_.isKnown(*it));

	dmce::OccupancyMap map(length, resolution, position, frameId, 98);
	EXPECT_TRUE(map.isKnown(*it));

	map = dmce::OccupancyMap(length, resolution, position, frameId, 3);
	EXPECT_TRUE(map.isKnown(*it));
}

TEST_F(TestOccupancyMap, ToCostMap_runs) {
	auto cmap = occMap_.toCostmap();
}

TEST_F(TestOccupancyMap, ToImage_runs) {
	auto image = occMap_.toImage();
}

TEST_F(TestOccupancyMap, GetFrontier) {
	dmce::indexList_t frontier = occMap_.getFrontier();
	EXPECT_EQ(frontier.size(), 0);
}

TEST_F(TestOccupancyMap, isIndexInRange) {
	auto maxIdx = length / resolution;
	EXPECT_TRUE(occMap_.isIndexInRange({0,0}));
	EXPECT_TRUE(occMap_.isIndexInRange({maxIdx.x()-1,0}));
	EXPECT_TRUE(occMap_.isIndexInRange({0,maxIdx.y()-1}));

	EXPECT_FALSE(occMap_.isIndexInRange({-1,0}));
	EXPECT_FALSE(occMap_.isIndexInRange({maxIdx.x(),0}));
	EXPECT_FALSE(occMap_.isIndexInRange({0,-1}));
	EXPECT_FALSE(occMap_.isIndexInRange({0,maxIdx.y()}));
}


TEST_F(TestOccupancyMap, getNCells_isCorrect) {
	auto size = length / resolution;
	unsigned int expectedNCells = size.x() * size.y();
	EXPECT_EQ(occMap_.getNCells(), expectedNCells);
}

TEST_F(TestOccupancyMap, getNUnknownCells_isCorrect) {
	EXPECT_EQ(occMap_.getNUnknownCells(), occMap_.getNCells());
}

TEST_F(TestOccupancyMap, onInitialise_entropy_isCorrect) {
	EXPECT_EQ(occMap_.getRelativeEntropy(), 1.0);

	dmce::OccupancyMap occMap_free(
			length, resolution, position, frameId, 1
		);
	EXPECT_EQ(occMap_free.getRelativeEntropy(), 0.0);

	dmce::OccupancyMap occMap_occ(
			length, resolution, position, frameId, 99
		);
	EXPECT_EQ(occMap_occ.getRelativeEntropy(), 0.0);
}

TEST_F(TestOccupancyMap, getSubmapMsg) {
	bool success = true;
	grid_map_msgs::GridMap msg = occMap_.getSubmapMessage({0,0}, {5,5}, success);
	EXPECT_TRUE(success);
}

TEST_F(TestOccupancyMap, fromImage) {
	auto image = occMap_.toImage();
	dmce::OccupancyMap newMap;
	newMap.fromImage(image, resolution, occMap_.getPosition());
	EXPECT_DOUBLE_EQ(newMap.getPosition().x(), occMap_.getPosition().x());
	EXPECT_DOUBLE_EQ(newMap.getPosition().y(), occMap_.getPosition().y());
}

}
