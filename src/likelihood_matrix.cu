
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
        gaussian_op(Point _mu, double _weight, double _det, std::array<double,9> _inv_of_covariance) :
            mu(_mu),
            base(_weight / sqrt(pow(2*M_PI, 3) * _det)),
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

        arma::mat33 arma_inv_of_covariance = arma::inv(arma_cov_mat);

        std::array<double,9> inv_of_covariance;
        for(auto i = 0; i < 3; i++) {
            for (auto j=0; j < 3; j++) {
                inv_of_covariance[i*3+j] = arma_inv_of_covariance(i,j);
            }
        }

        gaussian_op op(distribution.get_mu(), distribution.get_weight(), det_of_covariance, inv_of_covariance);

        thrust::transform(pcl.begin(), pcl.end(), result, op);
    }

    void LikelihoodMatrix::normalize_likelihoods(int n_points,
                                                 thrust::device_vector<double>& likelihoods) {
        typedef StridedRange<thrust::device_vector<double>::iterator> strided_iterator;

        for(int i = 0; i < n_points; i++) {
            strided_iterator iterator(likelihoods.begin() + i, likelihoods.end(), n_points);

            double sum_of_likelihoods = thrust::reduce(iterator.begin(), iterator.end(),
                                                       0.0, thrust::plus<double>());

            thrust::transform(iterator.begin(), iterator.end(),
                              thrust::make_constant_iterator(sum_of_likelihoods),
                              iterator.begin(), thrust::divides<double>());
        }
    }



    GaussianMixture LikelihoodMatrix::gaussian_mixture_of_pcl(const PointCloud& pcl) const {
        std::vector<WeightedGaussian> gaussians;
        thrust::device_vector<Point> mus(n_distributions);

        for(int i = 0; i < n_distributions; i++) {
            double sum_of_gammas = thrust::reduce(likelihoods.begin() + i*n_points,
                                                  likelihoods.begin() + (i+1)*n_points,
                                                  0.0, thrust::plus<double>());

            Point new_mu = compute_mu(pcl, likelihoods.begin() + i*n_points, sum_of_gammas);
            CovarianceMatrix new_sigma = compute_sigma(pcl,
                                                       likelihoods.begin() + i*n_points,
                                                       sum_of_gammas,
                                                       new_mu);

            double new_weight = sum_of_gammas / n_points;

            gaussians.push_back(WeightedGaussian(new_mu, new_sigma, new_weight));
        }

        return GaussianMixture(gaussians);
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

    Point LikelihoodMatrix::compute_mu(const PointCloud& pcl,
                                       thrust::device_vector<double>::const_iterator likelihoods,
                                       double sum_of_gammas) const {
        Point new_mu = thrust::inner_product(pcl.begin(), pcl.end(),
                                             likelihoods,
                                             Point(0.0, 0.0, 0.0),
                                             sum_of_pts_op(),
                                             point_by_scalar_op());

        new_mu = Point(new_mu.x / sum_of_gammas,
                       new_mu.y / sum_of_gammas,
                       new_mu.z / sum_of_gammas);

        return new_mu;
    }

    struct sum_of_cov_matrix_op : public thrust::binary_function<RawCovarianceMatrix, RawCovarianceMatrix, RawCovarianceMatrix> {
    public:
        __host__ __device__
        RawCovarianceMatrix operator()(RawCovarianceMatrix lhs, RawCovarianceMatrix rhs) {
            return lhs + rhs;
        }
    };

    struct point_to_cov_op : public thrust::binary_function<Point,double,RawCovarianceMatrix> {
    public:
        __host__ __device__
        RawCovarianceMatrix operator()(Point& p, double gamma) {
            RawCovarianceMatrix r;

            r.v00 = p.x * p.x * gamma;
            r.v11 = p.y * p.y * gamma;
            r.v22 = p.z * p.z * gamma;

            r.v10 = r.v01 = p.x * p.y * gamma;
            r.v20 = r.v02 = p.x * p.z * gamma;
            r.v21 = r.v12 = p.y * p.z * gamma;

            return r;
        }
    };

    CovarianceMatrix LikelihoodMatrix::compute_sigma(const PointCloud& pcl,
                                                        thrust::device_vector<double>::const_iterator likelihoods,
                                                        double sum_of_gammas,
                                                        const Point& new_mu) const {
        RawCovarianceMatrix init = RawCovarianceMatrix::zeros();
        RawCovarianceMatrix new_sigma = thrust::inner_product(pcl.begin(), pcl.end(),
                                                              likelihoods,
                                                              init,
                                                              sum_of_cov_matrix_op(),
                                                              point_to_cov_op());

        RawCovarianceMatrix base_sigma;

        base_sigma.v00 = new_mu.x * new_mu.x;
        base_sigma.v11 = new_mu.y * new_mu.y;
        base_sigma.v22 = new_mu.z * new_mu.z;

        base_sigma.v01 = base_sigma.v10 = new_mu.x * new_mu.y;
        base_sigma.v02 = base_sigma.v20 = new_mu.x * new_mu.z;
        base_sigma.v21 = base_sigma.v12 = new_mu.z * new_mu.y;

        new_sigma = new_sigma / sum_of_gammas - base_sigma;

        return CovarianceMatrix(new_sigma);
    }
}
