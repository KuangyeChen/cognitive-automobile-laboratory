#ifndef SAFE_ITERATOR_OPERATIONS_H
#define SAFE_ITERATOR_OPERATIONS_H

namespace lateral_control_ros_tool {

#include <iterator>

template <typename _ForwardIterator>
inline _ForwardIterator safe_next(_ForwardIterator __x,
                                  typename std::iterator_traits<_ForwardIterator>::difference_type __n,
                                  _ForwardIterator __last) {
    if (std::distance(__x, __last) < __n) {
        return __last;
    }
    std::advance(__x, __n);
    return __x;
}

template <typename _BidirectionalIterator>
inline _BidirectionalIterator safe_prev(_BidirectionalIterator __x,
                                        typename std::iterator_traits<_BidirectionalIterator>::difference_type __n,
                                        _BidirectionalIterator __begin) {
    if (std::distance(__x, __begin) < __n) {
        return __begin;
    }
    std::advance(__x, -__n);
    return __x;
}

}

#endif // SAFE_ITERATOR_OPERATIONS_H
