#pragma once

#include "dmce_core/OccupancyMap.hpp"

#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>

namespace dmce {
	/**
	 * An OccupancyMap that can be initialised from an image file.
	 */
	class GroundTruthMap : public OccupancyMap {
	public:
		// Inherit constructors
		using OccupancyMap::OccupancyMap;

		/**
		 * Constructor. The map is initialised from the image file at imagePath.
		 */
		GroundTruthMap(const std::string& imagePath,
				 const double& resolution,
				 const std::string& frameId = "map");
	};
};
