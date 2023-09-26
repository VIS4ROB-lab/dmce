#include "dmce_sim/GroundTruthMap.hpp"

#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui.hpp>

namespace dmce {
	GroundTruthMap::GroundTruthMap(const std::string& imagePath,
			 const double& resolution,
			 const std::string& frameId)
	{
		cv::Mat image = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);
		if (image.data == NULL) {
			throw std::runtime_error(
				"[GroundTruthMap] Image file '" + imagePath + "' could not be opened."
			);
		}
		
		cv::rotate(image, image, cv::ROTATE_90_COUNTERCLOCKWISE);
		this->fromImage(image, resolution);
		map_.setFrameId(frameId);

		auto it = getGridMapIterator();
		for ( ; !it.isPastEnd(); ++it) {
			if (std::abs(getOccupancy(*it) - 50) < 5) {
				map_.at("occupancy", *it) = 50;
			}
		}
	}
};
