#ifndef STRIDED_RANGE_H
#define STRIDED_RANGE_H

#include <thrust/functional.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/iterator/transform_iterator.h>
#include <thrust/iterator/permutation_iterator.h>

namespace pclem {

    template <typename T>
        class StridedRange {
    public:
        typedef typename thrust::iterator_difference<T>::type index_type;

        struct stride_op : public thrust::unary_function<index_type, index_type> {
            index_type stride;

            stride_op(index_type stride) : stride(stride) {}

            __host__ __device__
            index_type operator()(const index_type& i) {
                return i * stride;
            }
        };

        typedef typename thrust::counting_iterator<index_type> counting_iterator;
        typedef typename thrust::transform_iterator<stride_op, counting_iterator> transform_iterator;
        typedef typename thrust::permutation_iterator<T,transform_iterator> strided_iterator;

        StridedRange(T first, T last, index_type stride)
            : first(first), last(last), stride(stride) {}

        strided_iterator begin(void) const {
            return strided_iterator(first, transform_iterator(counting_iterator(0), stride_op(stride)));
        }

        strided_iterator end(void) const {
            return begin() + ((last - first) + (stride - 1)) / stride;
        }

    private:
        T first, last;
        index_type stride;
    };

}
#endif
