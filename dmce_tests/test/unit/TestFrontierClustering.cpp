#include "dmce_test/common.hpp"

#include "dmce_core/RobotMap.hpp"
#include "dmce_core/FrontierClustering.hpp"

namespace dmce_test {

	using namespace dmce;
	using idx_t = grid_map::Index;

	class TestFrontierClustering : public testing::Test {
	public:
		RobotMap map;
		FrontierClustering fc;

		TestFrontierClustering()
			: map({50,50}, 0.1, {0,0}, "map")
		{ }

		void free3x3Block(index_t center) {
			// WARNING: does no bounds checking!
			for (int dx = -1; dx <= 1; dx++)
				for (int dy = -1; dy <= 1; dy++)
					map.setOccupancy(idx_t{center.x()+dx,center.y()+dy}, map.freeValue);
		}
	};

	TEST_F(TestFrontierClustering, OnParseBlankMap_NoClusters) {
		EXPECT_EQ(fc.getNClusters(), 0);
		fc.fromMapFrontiers(map);
		EXPECT_EQ(fc.getNClusters(), 0);
	}

	TEST_F(TestFrontierClustering, OnOneFree_ParseMap_OneCluster) {
		idx_t idx{1,1};
		map.setOccupancy(idx, map.freeValue);
		fc.fromMapFrontiers(map);
		EXPECT_EQ(fc.getNClusters(), 1);
		std::vector<pos_t> centroidPositions = fc.getCentroidPositions();
		EXPECT_EQ(centroidPositions.size(), 1);
		pos_t pos; map.getPosition(idx, pos);
		EXPECT_FLOAT_EQ(centroidPositions[0].x(), pos.x());
		EXPECT_FLOAT_EQ(centroidPositions[0].y(), pos.y());
		std::vector<unsigned int> clusterSizes = fc.getClusterSizes();
		EXPECT_EQ(clusterSizes.size(), 1);
		EXPECT_EQ(clusterSizes[0], 1);
	}

	TEST_F(TestFrontierClustering, OnTwoFree_DoubleParseMap_TwoClusters) {
		idx_t idx1{1,1}, idx2{1,3};
		map.setOccupancy(idx1, map.freeValue);
		fc.fromMapFrontiers(map);
		map.setOccupancy(idx2, map.freeValue);
		fc.fromMapFrontiers(map);
		EXPECT_EQ(fc.getNClusters(), 2);
		std::vector<pos_t> centroidPositions = fc.getCentroidPositions();
		EXPECT_EQ(centroidPositions.size(), 2);
		pos_t pos1; map.getPosition(idx1, pos1);
		EXPECT_FLOAT_EQ(centroidPositions[0].x(), pos1.x());
		EXPECT_FLOAT_EQ(centroidPositions[0].y(), pos1.y());
		pos_t pos2; map.getPosition(idx2, pos2);
		EXPECT_FLOAT_EQ(centroidPositions[1].x(), pos2.x());
		EXPECT_FLOAT_EQ(centroidPositions[1].y(), pos2.y());
		std::vector<unsigned int> clusterSizes = fc.getClusterSizes();
		EXPECT_EQ(clusterSizes.size(), 2);
		EXPECT_EQ(clusterSizes[0], 1);
		EXPECT_EQ(clusterSizes[1], 1);
	}

	TEST_F(TestFrontierClustering, On3x3Free_ParseMap_OneCluster) {
		free3x3Block(index_t{2,2});
		fc.fromMapFrontiers(map);
		EXPECT_EQ(fc.getNClusters(), 1);
		std::vector<pos_t> centroidPositions = fc.getCentroidPositions();
		EXPECT_EQ(centroidPositions.size(), 1);
		pos_t pos; map.getPosition({2,2}, pos);
		EXPECT_FLOAT_EQ(centroidPositions[0].x(), pos.x());
		EXPECT_FLOAT_EQ(centroidPositions[0].y(), pos.y());
		std::vector<unsigned int> clusterSizes = fc.getClusterSizes();
		EXPECT_EQ(clusterSizes.size(), 1);
		EXPECT_EQ(clusterSizes[0], 8);

		indexList_t expectedIndices{{1,1},{1,2},{1,3},{2,1},{2,3},{3,1},{3,2},{3,3}};
		auto actualIndices = fc.getCellsByCluster(0);
		ASSERT_EQ(actualIndices.size(), expectedIndices.size());
		std::sort(expectedIndices.begin(), expectedIndices.end(), std::less<index_t>());
		std::sort(actualIndices.begin(), actualIndices.end(), std::less<index_t>());
		for (unsigned int i = 0; i < expectedIndices.size(); i++) {
			ASSERT_EQ(expectedIndices[i].x(), actualIndices[i].x());
			ASSERT_EQ(expectedIndices[i].y(), actualIndices[i].y());
		}
	}

	TEST_F(TestFrontierClustering, On3x3FreeInCorner_ParseMap_OneCluster) {
		free3x3Block(index_t{1,1});
		fc.fromMapFrontiers(map);
		EXPECT_EQ(fc.getNClusters(), 1);
		std::vector<pos_t> centroidPositions = fc.getCentroidPositions();
		EXPECT_EQ(centroidPositions.size(), 1);
		std::vector<unsigned int> clusterSizes = fc.getClusterSizes();
		EXPECT_EQ(clusterSizes.size(), 1);
		EXPECT_EQ(clusterSizes[0], 5);
	}

	TEST_F(TestFrontierClustering, OnTwo3x3Free_ParseMap_TwoClusters) {
		idx_t idx1{2,2}, idx2{6,2};
		free3x3Block(idx1);
		free3x3Block(idx2);
		fc.fromMapFrontiers(map);
		EXPECT_EQ(fc.getNClusters(), 2);
		std::vector<pos_t> centroidPositions = fc.getCentroidPositions();
		EXPECT_EQ(centroidPositions.size(), 2);
		pos_t pos1; map.getPosition(idx1, pos1);
		EXPECT_FLOAT_EQ(centroidPositions[0].x(), pos1.x());
		EXPECT_FLOAT_EQ(centroidPositions[0].y(), pos1.y());
		pos_t pos2; map.getPosition(idx2, pos2);
		EXPECT_FLOAT_EQ(centroidPositions[1].x(), pos2.x());
		EXPECT_FLOAT_EQ(centroidPositions[1].y(), pos2.y());
		std::vector<unsigned int> clusterSizes = fc.getClusterSizes();
		EXPECT_EQ(clusterSizes.size(), 2);
		EXPECT_EQ(clusterSizes[0], 8);
		EXPECT_EQ(clusterSizes[1], 8);

		for (unsigned int i = 0; i < fc.getNClusters(); i++) {
			auto cells = fc.getCellsByCluster(i);
			EXPECT_EQ(cells.size(), clusterSizes[i]);
		}
	}

	TEST_F(TestFrontierClustering, OnUShape_ParseMap_OneCluster) {
		// Asymmetrical U shape
		//   #
		// # #
		// ###
		std::vector<index_t> toFree = {{0,2},{1,0},{1,2},{2,0},{2,1},{2,2}};
		pos_t expectedCentroid{0,0};
		for (const index_t& idx : toFree) {
			map.setOccupancy(idx, map.freeValue);
			pos_t pos; map.getPosition(idx, pos);
			expectedCentroid += pos;
		}
		expectedCentroid /= toFree.size();
		fc.fromMapFrontiers(map);
		EXPECT_EQ(fc.getNClusters(), 1);
		std::vector<pos_t> centroidPositions = fc.getCentroidPositions();
		EXPECT_FLOAT_EQ(centroidPositions[0].x(), expectedCentroid.x());
		EXPECT_FLOAT_EQ(centroidPositions[0].y(), expectedCentroid.y());
	}

	TEST_F(TestFrontierClustering, OnParseMap_indexToPosition_isCorrect) {
		fc.fromMapFrontiers(map);
		std::vector<index_t> toCheck = {{0,0},{1,1},{2,4},{45,22}};
		for (const index_t& index : toCheck) {
			pos_t expectedPos; map.getPosition(index, expectedPos);
			pos_t returnedPos = fc.indexToPosition(index);
			ASSERT_FLOAT_EQ(expectedPos.x(), returnedPos.x());
			ASSERT_FLOAT_EQ(expectedPos.y(), returnedPos.y());
		}
	}

	TEST_F(TestFrontierClustering, OnFreeCircle_ParseMap_isCorrect) {
		pos_t expectedCentroid = {-1, 2};
		auto it = map.getCircleIterator(expectedCentroid, 5);
		for ( ; !it.isPastEnd(); ++it)
			map.setOccupancy(*it, map.freeValue);
		fc.fromMapFrontiers(map);
		EXPECT_EQ(fc.getNClusters(), 1);
		EXPECT_NEAR(fc.getCentroidPositions()[0].x(), expectedCentroid.x(), 1e-9);
		EXPECT_NEAR(fc.getCentroidPositions()[0].y(), expectedCentroid.y(), 1e-9);
	}

	TEST_F(TestFrontierClustering, MessageConversion) {
		idx_t idx1{2,2}, idx2{6,2};
		free3x3Block(idx1);
		free3x3Block(idx2);
		fc.fromMapFrontiers(map);
		auto msg = fc.getMessage();
		FrontierClustering newFc;
		newFc.fromMessage(msg);

		// Check centroids and cluster sizes
		EXPECT_EQ(fc.getNClusters(), newFc.getNClusters());
		auto expectedCentroids = fc.getCentroidPositions();
		auto expectedSizes = fc.getClusterSizes();
		auto actualCentroids = newFc.getCentroidPositions();
		auto actualSizes = newFc.getClusterSizes();
		ASSERT_EQ(fc.getNClusters(), actualCentroids.size());
		ASSERT_EQ(fc.getNClusters(), actualSizes.size());
		for (unsigned int i = 0; i < fc.getNClusters(); ++i) {
			ASSERT_FLOAT_EQ(expectedCentroids[i].x(), actualCentroids[i].x());
			ASSERT_FLOAT_EQ(expectedCentroids[i].y(), actualCentroids[i].y());
			ASSERT_EQ(expectedSizes[i], actualSizes[i]);
		}

		// Check contents of each cluster
		for (unsigned int j = 0; j < fc.getNClusters(); ++j) {
			auto expectedIndices = fc.getCellsByCluster(j);
			auto actualIndices = newFc.getCellsByCluster(j);
			ASSERT_EQ(expectedIndices.size(), actualIndices.size());
			for (unsigned int i = 0; i < expectedIndices.size(); i++) {
				ASSERT_EQ(expectedIndices[i].x(), actualIndices[i].x());
				ASSERT_EQ(expectedIndices[i].y(), actualIndices[i].y());
			}
		}

		// Check index-to-position conversions
		std::vector<index_t> toCheck = {{0,0},{1,1},{2,4},{45,22}};
		for (const index_t& index : toCheck) {
			pos_t expectedPos; map.getPosition(index, expectedPos);
			pos_t returnedPos = newFc.indexToPosition(index);
			ASSERT_FLOAT_EQ(expectedPos.x(), returnedPos.x());
			ASSERT_FLOAT_EQ(expectedPos.y(), returnedPos.y());
		}
	}

	TEST_F(TestFrontierClustering, TestSortingByDistance) {
		idx_t idx1{2,2}, idx2{6,2}, idx3{-2,5};
		free3x3Block(idx1);
		free3x3Block(idx2);
		free3x3Block(idx3);
		fc.fromMapFrontiers(map);

		pos_t refPos{9, 8};
		auto sortedClusters = fc.sortClustersByDistance(refPos);
		ASSERT_EQ(sortedClusters.size(), fc.getNClusters());
		auto clusterCentroids = fc.getCentroidPositions();
		for (unsigned int i = 0; i < fc.getNClusters(); i++) {
			unsigned int clusterId = sortedClusters[i].first;
			double expected = (clusterCentroids[clusterId] - refPos).squaredNorm();
			ASSERT_FLOAT_EQ(sortedClusters[i].second, expected);
		}

		for (unsigned int i = 1; i < fc.getNClusters(); i++) {
			ASSERT_TRUE(sortedClusters[i-1].second <= sortedClusters[i].second);
		}
	}
}

