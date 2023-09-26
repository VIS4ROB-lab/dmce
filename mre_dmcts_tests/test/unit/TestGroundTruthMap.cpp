#include "mre_dmcts_test/common.hpp"
#include <ros/package.h>

#include "mre_dmcts_sim/GroundTruthMap.hpp"

namespace mre_dmcts_test {

std::string packagePath = ros::package::getPath("mre_dmcts_sim") + "/";

TEST(TestGroundTruthMap, OnNonexistentFile_ThrowsError) {
	EXPECT_THROW(
		mre_dmcts::GroundTruthMap(packagePath+"/maps/nonExistentFile.png", 1, "map"),
		std::runtime_error)
	<< "GroundTruthMap should throw an error if the file is not found.";
}

TEST(TestGroundTruthMap, ReadImageCorrectly) {
	mre_dmcts::GroundTruthMap map(packagePath + "maps/testMap.png", 1, "map");
	Funcs::checkTestMap(map, 1);
	EXPECT_DOUBLE_EQ(map.getRelativeEntropy(), 1./3.);
}

}
