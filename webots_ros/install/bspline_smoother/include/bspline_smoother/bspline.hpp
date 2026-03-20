// bspline.hpp
// Deklariert die BSpline-Klasse für global interpolierende kubische
// B-Splines. Die Implementierung folgt der Shene-Notation (CS3621).
//
// Quellen:
//   [SHENE_INT]   Shene, C.-K. (2014). Global Cubic Spline Interpolation.
//                 https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/
//                 INT-APP/CURVE-INT-global.html
//   [SHENE_BASIS] Shene, C.-K. (2014). B-Spline Basis Functions.
//                 https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/
//                 spline/B-spline/bspline-basis.html

#ifndef BSPLINE_SMOOTHER__BSPLINE_HPP_
#define BSPLINE_SMOOTHER__BSPLINE_HPP_

#include <vector>

#include <Eigen/Dense>

namespace bspline_smoother {

// Einfacher 2D-Punkt. Wird im gesamten bspline_smoother-Paket verwendet.
struct Point2D {
  double x{0.0};
  double y{0.0};
};

// ---------------------------------------------------------------------------
// BSpline
// Interpolierender kubischer B-Spline (Grad p=3).
// Der Konstruktor führt die vollständige Pipeline aus:
//   1. Chord-Length-Parametrisierung  [SHENE_CHORD]
//   2. Knotenvektor (Averaging-Regel) [SHENE_KNOT]
//   3. Lösung von N·P = D via QR     [SHENE_INT]
// ---------------------------------------------------------------------------
class BSpline {
 public:
  // Konstruktor. Wirft std::invalid_argument wenn data_pts.size() < degree+1.
  explicit BSpline(const std::vector<Point2D>& data_pts, int degree);

  // Wertet C(u) für u ∈ [0,1] aus.
  Point2D Evaluate(double u) const;

  // Gleichmäßige Abtastung mit num_samples Punkten in [0,1].
  std::vector<Point2D> Sample(int num_samples) const;

 private:
  // Chord-Length-Parametrisierung: ordnet jedem Datenpunkt t_k ∈ [0,1] zu.
  static std::vector<double> ChordLength(const std::vector<Point2D>& pts);

  // Erstellt den Knotenvektor nach der Averaging-Regel [SHENE_KNOT].
  void BuildKnotVector(const std::vector<double>& t, int n, int p);

  // Löst das Interpolationssystem N·P = D [SHENE_INT].
  void SolveInterpolation(
    const std::vector<Point2D>& D,
    const std::vector<double>& t);

  // Cox–de-Boor-Rekursion: berechnet N_{i,p}(u) [SHENE_BASIS].
  double BasisFunction(int i, int p, double u) const;

  int degree_{3};
  std::vector<double>  knots_;           // Knotenvektor der Länge n+p+1
  std::vector<Point2D> control_points_;  // Kontrollpunkte P_0..P_{n-1}
};

}  // namespace bspline_smoother

#endif  // BSPLINE_SMOOTHER__BSPLINE_HPP_