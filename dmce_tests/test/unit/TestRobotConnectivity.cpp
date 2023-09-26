#include "dmce_test/common.hpp"

#include "dmce_msgs/RobotConnectivity.h"
#include "dmce_core/RobotConnectivity.hpp"

namespace dmce_test {
class TestRobotConnectivity : public ::testing::Test {
protected:
	unsigned int nRobots = 5;
	dmce::RobotConnectivity connectivity;

public:
	TestRobotConnectivity() : connectivity(nRobots) {

	}
};

TEST_F(TestRobotConnectivity, CheckLayout) {
	dmce::RobotConnectivity::Message msg = connectivity.getMessage();
	EXPECT_EQ(msg.data.size(), (nRobots+1)*(nRobots+1));
	EXPECT_EQ(msg.layout.dim[0].stride, (nRobots+1)*(nRobots+1));
	EXPECT_EQ(msg.layout.dim[0].size, nRobots+1);
	EXPECT_EQ(msg.layout.dim[1].stride, nRobots+1);
	EXPECT_EQ(msg.layout.dim[1].size, nRobots+1);
}

TEST_F(TestRobotConnectivity, CheckBounds) {
	dmce::RobotConnectivity::Message msg = connectivity.getMessage();

	ASSERT_THROW(connectivity.isConnected(0, 6), std::out_of_range);
	ASSERT_THROW(connectivity.isConnected(6, 0), std::out_of_range);

	ASSERT_THROW(connectivity.bothConnected(0, 6), std::out_of_range);
	ASSERT_THROW(connectivity.bothConnected(6, 0), std::out_of_range);

	ASSERT_THROW(connectivity.connect(0, 6), std::out_of_range);
	ASSERT_THROW(connectivity.connect(6, 0), std::out_of_range);

	ASSERT_THROW(connectivity.disconnect(0, 6), std::out_of_range);
	ASSERT_THROW(connectivity.disconnect(6, 0), std::out_of_range);
}

TEST_F(TestRobotConnectivity, CheckDefaultConnectivity) {
	dmce::RobotConnectivity::Message msg = connectivity.getMessage();

	for (unsigned int from = 0; from < nRobots+1; from++) {
		for (unsigned int to = 0; to < nRobots+1; to++) {
			ASSERT_TRUE(connectivity.isConnected(from, to));
		}
	}
}

TEST_F(TestRobotConnectivity, CheckConnectivityToggles) {
	dmce::RobotConnectivity::Message msg = connectivity.getMessage();

	for (unsigned int from = 0; from < nRobots+1; from++) {
		for (unsigned int to = 0; to < nRobots+1; to++) {
			ASSERT_TRUE(connectivity.isConnected(from, to));
			connectivity.disconnect(from, to);
			ASSERT_FALSE(connectivity.isConnected(from, to));
			connectivity.connect(from, to);
			ASSERT_TRUE(connectivity.isConnected(from, to));
		}
	}
}

TEST_F(TestRobotConnectivity, CheckTwoWayConnectivityToggles) {
	dmce::RobotConnectivity::Message msg = connectivity.getMessage();

	for (unsigned int from = 0; from < nRobots+1; from++) {
		for (unsigned int to = from+1; to < nRobots+1; to++) {
			ASSERT_TRUE(connectivity.isConnected(from, to));
			connectivity.disconnectBoth(from, to);
			ASSERT_FALSE(connectivity.isConnected(from, to));
			ASSERT_FALSE(connectivity.isConnected(to, from));
			connectivity.connectBoth(from, to);
			ASSERT_TRUE(connectivity.bothConnected(from, to));
		}
	}
}

TEST_F(TestRobotConnectivity, CheckMessageConstructor) {
	connectivity.disconnect(0, 1);
	connectivity.disconnect(1, nRobots);

	dmce::RobotConnectivity::Message msg = connectivity.getMessage();
	dmce::RobotConnectivity copy(msg);

	ASSERT_EQ(copy.getNRobots(), connectivity.getNRobots());

	auto copyMsg = copy.getMessage();
	EXPECT_EQ(copyMsg.data.size(), (nRobots+1)*(nRobots+1));
	EXPECT_EQ(copyMsg.layout.dim[0].stride, (nRobots+1)*(nRobots+1));
	EXPECT_EQ(copyMsg.layout.dim[0].size, nRobots+1);
	EXPECT_EQ(copyMsg.layout.dim[1].stride, nRobots+1);
	EXPECT_EQ(copyMsg.layout.dim[1].size, nRobots+1);

	for (unsigned int from = 0; from < nRobots+1; from++) {
		for (unsigned int to = 0; to < nRobots+1; to++) {
			if ((from==0 && to==1) || (from==1 && to==nRobots)) {
				ASSERT_FALSE(copy.isConnected(from, to));
			} else {
				ASSERT_TRUE(copy.isConnected(from, to));
			}
		}
	}
}

TEST_F(TestRobotConnectivity, CheckBothConnected) {
	EXPECT_TRUE(connectivity.bothConnected(0, 1));
	connectivity.disconnect(0, 1); // 1->0 only
	EXPECT_FALSE(connectivity.bothConnected(0, 1));
	connectivity.disconnect(1, 0); // both directions disconnected
	EXPECT_FALSE(connectivity.bothConnected(0, 1));
	connectivity.connect(0, 1); // 0->1 only
	EXPECT_FALSE(connectivity.bothConnected(0, 1));
	connectivity.connect(1, 0); // both reconnected
	EXPECT_TRUE(connectivity.bothConnected(0, 1));
}
}
