#include "hausdorff.h"
#include "../../hausdorff.h"
#include <functional>

template <
  typename DerivedV,
  typename Kernel,
  typename Scalar>
IGL_INLINE void igl::copyleft::cgal::hausdorff(
  const Eigen::MatrixBase<DerivedV>& V,
  const CGAL::AABB_tree<
    CGAL::AABB_traits<Kernel, 
      CGAL::AABB_triangle_primitive<Kernel, 
        typename std::vector<CGAL::Triangle_3<Kernel> >::iterator
      >
    >
  > & treeB,
  const std::vector<CGAL::Triangle_3<Kernel> > & /*TB*/,
  Scalar & l,
  Scalar & u)
{
  // Not sure why using `auto` here doesn't work with the `hausdorff` function
  // parameter but explicitly naming the type does...
  const std::function<double(const double &,const double &,const double &)> 
    dist_to_B = [&treeB](
    const double & x, const double & y, const double & z)->double
  {
    CGAL::Point_3<Kernel> query(x,y,z);
    typename CGAL::AABB_tree<
      CGAL::AABB_traits<Kernel, 
        CGAL::AABB_triangle_primitive<Kernel, 
          typename std::vector<CGAL::Triangle_3<Kernel> >::iterator
        >
      >
    >::Point_and_primitive_id pp = treeB.closest_point_and_primitive(query);
    return std::sqrt((query-pp.first).squared_length());
  };
  return igl::hausdorff(V,dist_to_B,l,u);
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template instantiation
template void igl::copyleft::cgal::hausdorff<Eigen::Matrix<double, -1, -1, 0, -1, -1>, CGAL::Simple_cartesian<double>, double>(Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, CGAL::AABB_tree<CGAL::AABB_traits<CGAL::Simple_cartesian<double>, CGAL::AABB_triangle_primitive<CGAL::Simple_cartesian<double>, std::vector<CGAL::Triangle_3<CGAL::Simple_cartesian<double> >, std::allocator<CGAL::Triangle_3<CGAL::Simple_cartesian<double> > > >::iterator, CGAL::Boolean_tag<false> >, CGAL::Default> > const&, std::vector<CGAL::Triangle_3<CGAL::Simple_cartesian<double> >, std::allocator<CGAL::Triangle_3<CGAL::Simple_cartesian<double> > > > const&, double&, double&);
#endif