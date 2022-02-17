#include "algorithm.hpp"
#include "curvature_label.hpp"
#include "index_range.hpp"

template<typename PointT>
std::vector<CurvatureLabel> Label(
    const Mask<PointT> & input_mask,
    const Range<PointT> & range,
    const int n_blocks,
    const int padding,
    const int max_edges_per_block)
{
  Mask mask = input_mask;

  std::vector<CurvatureLabel> labels(mask.Size(), CurvatureLabel::Default);
  const PaddedIndexRange index_range(0, mask.Size(), n_blocks, padding);
  for (int j = 0; j < n_blocks; j++) {
    const std::vector<double> ranges = range(index_range.Begin(j), index_range.End(j));
    const std::vector<double> curvature = CalcCurvature(ranges);
    const std::vector<int> indices = Argsort(curvature);

    const int expected_size = index_range.End(j) - index_range.Begin(j) - 2 * padding;
    assert(curvature.size() == static_cast<std::uint32_t>(expected_size));

    const int offset = index_range.Begin(j) + padding;

    int n_picked = 0;
    for (const int index : boost::adaptors::reverse(indices)) {
      if (mask.At(offset + index) || curvature.at(index) <= edge_threshold) {
        continue;
      }

      if (n_picked >= max_edges_per_block) {
        break;
      }

      n_picked++;

      labels.at(offset + index) = CurvatureLabel::Edge;

      mask.FillNeighbors(offset + index, padding);
    }

    for (const int index : indices) {
      if (mask.At(offset + index) || curvature.at(index) >= surface_threshold) {
        continue;
      }

      labels.at(offset + index) = CurvatureLabel::Surface;

      mask.FillNeighbors(offset + index, padding);
    }
  }
  return labels;
}

