#include "strided_range.h"

namespace pclem {


    // template <typename T>
    // StridedRange<T>::StridedRange(T fisrt, T last, StridedRange<T>::index_type stride)
    //     : first(first), last(last), stride(stride) {}

    template <typename T>
    typename StridedRange<T>::strided_iterator StridedRange<T>::begin() const {
        return permutation_iterator(first, transform_iterator(counting_iterator(0), stride_op(stride)));
    }

    template <typename T>
    typename StridedRange<T>::strided_iterator StridedRange<T>::end() const {
        return begin() + ((last - first) + (stride - 1)) / stride;
    }


}
