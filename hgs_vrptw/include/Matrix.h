#ifndef MATRIX_H
#define MATRIX_H

#include <vector>

// Implementation of a matrix in a C++ vector
// This class is used because a flat vector is faster than a vector of vectors
// which requires two lookup operations rather than one to index a matrix
// element
class Matrix
{
    size_t cols_;            // The number of columns of the matrix
    std::vector<int> data_;  // The vector where all the data is stored (this
                             // represents the matrix)

public:
    // Empty constructor: with zero columns and a vector of size zero
    Matrix() : cols_(0), data_(0) {}

    // Constructor: create a matrix of size dimension by dimension, using a C++
    // vector of size dimension * dimension.
    explicit Matrix(size_t dimension)
        : cols_(dimension), data_(dimension * dimension)
    {
    }

    [[nodiscard]] inline int &operator()(size_t row, size_t col)
    {
        return data_[cols_ * row + col];
    }

    [[nodiscard]] inline int operator()(size_t row, size_t col) const
    {
        return data_[cols_ * row + col];
    }

    [[nodiscard]] int max() const
    {
        return *std::max_element(data_.begin(), data_.end());
    }
};

#endif
