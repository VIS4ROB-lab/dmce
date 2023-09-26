#include "dmce_nodes/GroundTruthMapServer.hpp"

namespace dmce {
	GroundTruthMapServer::GroundTruthMapServer(ros::NodeHandle nh, unsigned int robotId)
		: NodeServer(nh, robotId)
	{
		double resolution;
		getRequiredParam("/globalMap/resolution", resolution);
		std::string imageFile;
		getRequiredParam("/globalMap/groundTruthImage", imageFile);

		map_ = GroundTruthMap(imageFile, resolution, "map");

		initStartingArea_();

		groundTruthMapServer_ = nodeHandle_.advertiseService(
			"GroundTruthMapService",
			&GroundTruthMapServer::groundTruthMapCallback_,
			this
		);

		startingAreaServer_ = nodeHandle_.advertiseService(
			"StartingAreaService",
			&GroundTruthMapServer::startingAreaCallback_,
			this
		);

	};

	bool GroundTruthMapServer::groundTruthMapCallback_(
		dmce_msgs::GetMap::Request& request,
		dmce_msgs::GetMap::Response& response)
	{
		response.map = map_.getMessage();
		return true;
	}

	bool GroundTruthMapServer::startingAreaCallback_(
		dmce_msgs::GetStartingArea::Request& request,
		dmce_msgs::GetStartingArea::Response& response)
	{
		response = startingArea_;
		return true;
	}

	void GroundTruthMapServer::initStartingArea_() {
		getRequiredParam("/robot/startingArea/position_x", startingArea_.position_x);
		getRequiredParam("/robot/startingArea/position_y", startingArea_.position_y);
		getRequiredParam("/robot/startingArea/width", startingArea_.width);
		getRequiredParam("/robot/startingArea/height", startingArea_.height);
		bool randomiseStartingArea;
		getRequiredParam("/robot/startingArea/randomisePosition", randomiseStartingArea);
		double requiredFreeSpace;
		getRequiredParam("/robot/startingArea/requiredFreeSpace", requiredFreeSpace);
		double robotDiameter;
		getRequiredParam("/robot/diameter", robotDiameter);

		if (!randomiseStartingArea) {
			return;
		}

		auto isValidGSPosition = [&robotDiameter](const pos_t& pos, const OccupancyMap& map) -> bool {
			auto it = map.getCircleIterator(pos, robotDiameter);
			for ( ; !it.isPastEnd(); ++it) {
				if (map.isOccupied(*it))
					return false;
			}
			return true;
		};

		auto area = startingArea_;
		auto isValid = [&requiredFreeSpace, &area, &isValidGSPosition](const pos_t& pos, const OccupancyMap& map) -> bool {
			if (!isValidGSPosition(pos, map))
				return false;

			auto it = map.getSubmapIterator(pos, {area.width, area.height});
			unsigned int freeCount = 0, totalCount = 0;
			for ( ; !it.isPastEnd(); ++it) {
				totalCount++;
				freeCount += !map.isOccupied(*it);
			}
			return (double(freeCount) / totalCount) >= requiredFreeSpace;
		};

		pos_t mapPos = map_.getPosition();
		auto mapLen = map_.getLength();
		double minx = mapPos.x() - mapLen.x() * .5;
		double miny = mapPos.y() - mapLen.y() * .5;
		double maxx = mapPos.x() + mapLen.x() * .5;
		double maxy = mapPos.y() + mapLen.y() * .5;

		auto gen = utils::RNG::get();
		std::uniform_real_distribution<> x_dis(minx, maxx);
		std::uniform_real_distribution<> y_dis(miny, maxy);
		pos_t position = {startingArea_.position_x, startingArea_.position_y};
		do {
			position = {x_dis(gen), y_dis(gen)};
		} while (!isValid(position, map_));
		startingArea_.position_x = position.x();
		startingArea_.position_y = position.y();
	}
};

