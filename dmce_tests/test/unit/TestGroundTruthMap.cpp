#include "dmce_test/common.hpp"
#include <ros/package.h>

#include "dmce_sim/GroundTruthMap.hpp"

namespace dmce_test {

std::string packagePath = ros::package::getPath("dmce_sim") + "/";

TEST(TestGroundTruthMap, OnNonexistentFile_ThrowsError) {
	EXPECT_THROW(
		dmce::GroundTruthMap(packagePath+"/maps/nonExistentFile.png", 1, "map"),
		std::runtime_error)
	<< "GroundTruthMap should throw an error if the file is not found.";
}

TEST(TestGroundTruthMap, ReadImageCorrectly) {
	dmce::GroundTruthMap map(packagePath + "maps/testMap.png", 1, "map");
	Funcs::checkTestMap(map, 1);
	EXPECT_DOUBLE_EQ(map.getRelativeEntropy(), 1./3.);
}

}
