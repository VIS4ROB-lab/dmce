
#include "mre_dmcts_core/utils.hpp"

namespace mre_dmcts {
namespace utils {

	bool RNG::isInitialised_ = false;
	RNG::generator_t RNG::gen_(0);

	void RNG::init(unsigned int seed) {
		if (isInitialised_)
			return;

		if (seed == 0) {
			std::random_device rd;
			seed = rd();
		}

		gen_ = generator_t(seed);
		isInitialised_ = true;
	}

	RNG::generator_t& RNG::get() {
		if (!isInitialised_)
			init();

		return gen_;
	}

} // namespace utils
} // namespace mre_dmcts
