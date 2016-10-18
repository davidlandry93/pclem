
#include <thrust/iterator/constant_iterator.h>
#include <thrust/inner_product.h>
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
            strided_iterator iterator(likelihoods.begin() + i, likelihoods.end(), n_points);

            double sum_of_likelihoods = thrust::reduce(iterator.begin(), iterator.end(), 0.0, thrust::plus<double>());

            thrust::transform(iterator.begin(), iterator.end(),
                              thrust::make_constant_iterator(sum_of_likelihoods),
                              iterator.begin(), thrust::divides<double>());
        }
    }

    struct point_by_scalar_op : public thrust::binary_function<Point,double,Point> {
    public:
        __host__ __device__
        Point operator()(Point x, double pi) {
            return Point(x.x * pi, x.y * pi, x.z * pi);
        }
    };

    struct sum_of_pts_op : public thrust::binary_function<Point,Point,Point> {
    public:
        __host__ __device__
        Point operator()(Point lhs, Point rhs) {
            return Point(lhs.x + rhs.x,
                         lhs.y + rhs.y,
                         lhs.z + rhs.z);
        }
    };

    GaussianMixture LikelihoodMatrix::gaussian_mixture_of_pcl(const PointCloud& pcl) const {
        std::vector<WeightedGaussian> gaussians;
        thrust::device_vector<Point> mus(n_distributions);

        for(int i = 0; i < n_distributions; i++) {
            Point new_mu = thrust::inner_product(pcl.begin(), pcl.end(),
                                                 likelihoods.begin() + i*n_points,
                                                 Point(0.0, 0.0, 0.0),
                                                 sum_of_pts_op(),
                                                 point_by_scalar_op());

            double sum_of_gammas = thrust::reduce(likelihoods.begin() + i*n_points,
                                                  likelihoods.begin() + (i+1)*n_points,
                                                  0.0, thrust::plus<double>());

            new_mu = Point(new_mu.x / sum_of_gammas,
                           new_mu.y / sum_of_gammas,
                           new_mu.z / sum_of_gammas);

            std::cout << "New mu: " << new_mu <<
                "Sum of gammas: " << sum_of_gammas << std::endl;
        }

        return GaussianMixture(gaussians);
    }
}
