#include "algorithm.hpp"
#include "curvature_label.hpp"
#include "index_range.hpp"

template<typename PointT>
void LabelEdges(
  std::vector<CurvatureLabel> & labels,
  Mask<PointT> & mask,
  const std::vector<double> curvature,
  const std::vector<int> & indices,
  const int padding,
  const int offset,
  const double edge_threshold,
  const int max_edges_per_block)
{
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
}

template<typename PointT>
void LabelSurface(
  std::vector<CurvatureLabel> & labels,
  Mask<PointT> & mask,
  const std::vector<double> curvature,
  const std::vector<int> & indices,
  const int padding,
  const int offset,
  const double surface_threshold)
{
  for (const int index : indices) {
    if (mask.At(offset + index) || curvature.at(index) >= surface_threshold) {
      continue;
    }

    labels.at(offset + index) = CurvatureLabel::Surface;

    mask.FillNeighbors(offset + index, padding);
  }
}

template<typename PointT>
std::vector<CurvatureLabel> Label(
    const Mask<PointT> & input_mask,
    const Range<PointT> & range,
    const int n_blocks,
    const int padding,
    const int max_edges_per_block,
    const double edge_threshold,
    const double surface_threshold)
{
  Mask mask = input_mask;  // copy to make argument const

  std::vector<CurvatureLabel> labels(mask.Size(), CurvatureLabel::Default);
  const PaddedIndexRange index_range(0, mask.Size(), n_blocks, padding);
  for (int j = 0; j < n_blocks; j++) {
    const std::vector<double> ranges = range(index_range.Begin(j), index_range.End(j));
    const std::vector<double> curvature = CalcCurvature(ranges, padding);
    const std::vector<int> indices = Argsort(curvature);

    const int expected_size = index_range.End(j) - index_range.Begin(j) - 2 * padding;
    assert(curvature.size() == static_cast<std::uint32_t>(expected_size));

    const int offset = index_range.Begin(j) + padding;

    LabelEdges(labels, mask, curvature, indices,
               padding, offset, edge_threshold, max_edges_per_block);

    LabelSurface(labels, mask, curvature, indices,
                 padding, offset, surface_threshold);
  }

  return labels;
}
