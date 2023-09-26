#include "dmce_test/common.hpp"

#include "dmce_core/RobotMap.hpp"
#include "dmce_sim/SensorEmulator.hpp"

namespace dmce_test {

TEST(TestSensorEmulator, InvalidRangeThrowsError) {
	EXPECT_THROW(
		dmce::SensorEmulator emulator(-1),
		std::invalid_argument)
	<< "SensorEmulator should not accept negative sensor ranges.";
}

TEST(TestSensorEmulator, UpdatePositionsAreInRange) {
	double sensorRange = .2;
	unsigned int rayCount = 36;
	dmce::SensorEmulator emulator(sensorRange, rayCount);
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
	dmce::RobotMap map;
	auto it = map.getGridMapIterator();
	for (; !it.isPastEnd(); ++it) {
		float v = std::rand() / float(RAND_MAX);
		map.setOccupancy(*it, v);
	}

	dmce::SensorEmulator emulator(sensorRange, rayCount, map);
	grid_map::Position startPos{0, 0};
	auto update = emulator.getMapUpdate(startPos);

	for (uint i = 0; i < update.length; i++) {
		grid_map::Position samplePos{update.x_positions[i], update.y_positions[i]};
		float expected = map.getOccupancy(samplePos);
		EXPECT_FLOAT_EQ(update.values[i], expected);
	}
}

}
