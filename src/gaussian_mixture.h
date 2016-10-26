
#include <memory>

#include "private_gaussian_mixture.h"

namespace pclem {
    class GaussianMixture {
    public:
        GaussianMixture();
    private:
        std::unique_ptr<PrivateGaussianMixture> mixture;
    };
}
