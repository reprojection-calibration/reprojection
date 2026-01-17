#pragma once

#include "types/eigen_types.hpp"

namespace reprojection::projection_functions {

// TODO(Jack): Split between source and header?
class ImageBounds {
   public:
    ImageBounds(double const min_width, double const max_width, double const min_height, double const max_height)
        : u_min_{min_width}, u_max_{max_width}, v_min_{min_height}, v_max_{max_height} {}

    ImageBounds(Array4d const& image_dimension)
        : ImageBounds(image_dimension[0], image_dimension[1], image_dimension[2], image_dimension[3]) {}

    template <typename T>
    bool InBounds(T const u, T const v) const {
        return u_min_ < u and u < u_max_ and v_min_ < v and v < v_max_;
    }

   private:
    double u_min_;
    double u_max_;
    double v_min_;
    double v_max_;
};

}  // namespace reprojection::projection_functions