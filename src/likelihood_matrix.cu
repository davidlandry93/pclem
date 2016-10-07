
#include <thrust/extrema.h>
#include <armadillo>
#include "likelihood_matrix.h"
#include "strided_range.h"

namespace pclem {
    LikelihoodMatrix::LikelihoodMatrix() : likelihoods() {

    }

    LikelihoodMatrix::LikelihoodMatrix(int n_points,
                                       int n_distributions,
                                       std::vector<double>& likelihoods)
        : n_points(n_points), n_distributions(n_distributions), likelihoods(likelihoods) {}

    LikelihoodMatrix::LikelihoodMatrix(int n_points,
                                       int n_distributions,
                                       thrust::device_vector<double>& likelihoods)
        : n_points(n_points), n_distributions(n_distributions), likelihoods(likelihoods) {}

    LikelihoodMatrix::LikelihoodMatrix(LikelihoodMatrix&& other) : likelihoods() {
        std::swap(likelihoods, other.likelihoods);
    }

    LikelihoodMatrix LikelihoodMatrix::build(const PointCloud& pcl, const GaussianMixture& mixture) {
        int n_points = pcl.get_n_points();
        int n_gaussians = mixture.n_gaussians();
        thrust::device_vector<double> likelihoods(n_points*n_gaussians);

        for(int i = 0; i < n_gaussians; i++) {
            likelihoods_of_distribution(pcl, mixture.get_gaussian(i), likelihoods.begin() + i*n_points);
        }

        normalize_likelihoods(n_points, likelihoods);

        return LikelihoodMatrix(n_points, n_gaussians, likelihoods);
    }

    LikelihoodMatrix& LikelihoodMatrix::operator=(LikelihoodMatrix&& other) {
        std::swap(likelihoods, other.likelihoods);
        return *this;
    }

    struct gaussian_op : public thrust::unary_function<Point,double> {
    public:
        gaussian_op(Point _mu, double _det, std::array<double,9> _inv_of_covariance) :
            mu(_mu),
            base(1 / sqrt(pow(2*M_PI, 3) * _det)),
            inv() {
            for(int i = 0; i < 9; i++) {
                inv[i] = _inv_of_covariance[i];
            }

            std::cout << _mu;
            std::cout << _det;
        }


        // https://en.wikipedia.org/wiki/Multivariate_normal_distribution#Density_function
        __host__ __device__
        double operator()(Point x) {
            Point x_minus_mu = x - mu;
            double temp_product[3] = {x_minus_mu.z*inv[6] + x_minus_mu.y*inv[3] + x_minus_mu.x*inv[0],
                                      x_minus_mu.z*inv[7] + x_minus_mu.y*inv[4] + x_minus_mu.x*inv[1],
                                      x_minus_mu.z*inv[8] + x_minus_mu.y*inv[5] + x_minus_mu.x*inv[2]};

            double scale_product =
                x_minus_mu.x*temp_product[0] +
                x_minus_mu.y*temp_product[1] +
                x_minus_mu.z*temp_product[2];

            return base * exp(-0.5 * scale_product);
        }

        __const__ Point mu;
        __const__ double base;
        double inv[9];
    };

    void LikelihoodMatrix::likelihoods_of_distribution(const PointCloud& pcl,
                                                       const WeightedGaussian& distribution,
                                                       thrust::device_vector<double>::iterator result) {
        std::array<double,9> cov_mat = distribution.get_sigma().as_array();

        arma::mat33 arma_cov_mat(cov_mat.data());
        double det_of_covariance = arma::det(arma_cov_mat);

        arma::mat33 arma_inv_of_covariance = arma::inv_sympd(arma_cov_mat);

        std::array<double,9> inv_of_covariance;
        for(auto i = 0; i < 3; i++) {
            for (auto j=0; j < 3; j++) {
                inv_of_covariance[i*3+j] = arma_inv_of_covariance(i,j);
            }
        }

        gaussian_op op(distribution.get_mu(), det_of_covariance, inv_of_covariance);

        thrust::transform(pcl.begin(), pcl.end(), result, op);
    }

    void LikelihoodMatrix::normalize_likelihoods(int n_points,
                                                 thrust::device_vector<double>& likelihoods) {
        typedef StridedRange<thrust::device_vector<double>::iterator> strided_iterator;

        for(int i = 0; i < n_points; i++) {
            std::cout << "Before iterator" << std::endl;
            strided_iterator iterator(likelihoods.begin() + i, likelihoods.end(), n_points);

            for(auto it = iterator.begin(); it != iterator.end(); it++) {
                std::cout << *it;
            }
            std::cout << std::endl;
        }
    }
}
