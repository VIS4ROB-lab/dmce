#include "mre_dmcts_test/common.hpp"

#include "mre_dmcts_core/RobotMap.hpp"
#include "mre_dmcts_sim/SensorEmulator.hpp"

namespace mre_dmcts_test {

TEST(TestSensorEmulator, InvalidRangeThrowsError) {
	EXPECT_THROW(
		mre_dmcts::SensorEmulator emulator(-1),
		std::invalid_argument)
	<< "SensorEmulator should not accept negative sensor ranges.";
}

TEST(TestSensorEmulator, UpdatePositionsAreInRange) {
	double sensorRange = .2;
	unsigned int rayCount = 36;
	mre_dmcts::SensorEmulator emulator(sensorRange, rayCount);
	grid_map::Position startPos{0, 0};
	auto update = emulator.getMapUpdate(startPos);

	for (uint i = 0; i < update.length; i++) {
		grid_map::Position samplePos{update.x_positions[i], update.y_positions[i]};
		double sampleDist = (samplePos - startPos).norm();
		EXPECT_LE(sampleDist, sensorRange);
	}
}

TEST(TestSensorEmulator, UpdateValuesAreCorrect) {
	double sensorRange = .2;
	unsigned int rayCount = 36;
	mre_dmcts::RobotMap map;
	auto it = map.getGridMapIterator();
	for (; !it.isPastEnd(); ++it) {
		float v = std::rand() / float(RAND_MAX);
		map.setOccupancy(*it, v);
	}

	mre_dmcts::SensorEmulator emulator(sensorRange, rayCount, map);
	grid_map::Position startPos{0, 0};
	auto update = emulator.getMapUpdate(startPos);

	for (uint i = 0; i < update.length; i++) {
		grid_map::Position samplePos{update.x_positions[i], update.y_positions[i]};
		float expected = map.getOccupancy(samplePos);
		EXPECT_FLOAT_EQ(update.values[i], expected);
	}
}

}
