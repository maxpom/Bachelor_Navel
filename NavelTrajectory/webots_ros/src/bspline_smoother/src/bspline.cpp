// bspline.cpp
// Implementiert einen global interpolierenden kubischen B-Spline.
// Löst das Gleichungssystem N·P = D via ColPivHouseholderQR (Eigen3).
//
// Algorithmus-Quellen:
//   [SHENE_INT]   Shene, C.-K. (2014). CS3621 – Global Cubic Spline
//                 Interpolation. MTU.
//                 https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/INT-APP/CURVE-INT-global.html
//   [SHENE_CHORD] Shene, C.-K. (2014). Chord-Length Parameterization.
//                 https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/INT-APP/PARA-chord-length.html
//   [SHENE_KNOT]  Shene, C.-K. (2014). Knot Generation.
//                 https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/INT-APP/PARA-knot-generation.html
//   [SHENE_BASIS] Shene, C.-K. (2014). B-Spline Basis Functions.
//                 https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-basis.html

#include "bspline_smoother/bspline.hpp"

#include <algorithm>
#include <numeric>
#include <stdexcept>

namespace bspline_smoother {

// ---------------------------------------------------------------------------
// Konstruktor – führt die vollständige Interpolations-Pipeline aus.
// Notation folgt [SHENE_INT]: D = Datenpunkte, t = Parameterwerte,
// p = Grad, N = Basismatrix, P = Kontrollpunkte.
// ---------------------------------------------------------------------------
BSpline::BSpline(const std::vector<Point2D>& data_pts, int degree) {
  degree_ = degree;
  const int n = static_cast<int>(data_pts.size());

  if (n < degree_ + 1) {
    throw std::invalid_argument(
      "Not enough data points for requested spline degree");
  }

  const auto t = ChordLength(data_pts);  // [SHENE_CHORD]
  BuildKnotVector(t, n, degree_);        // [SHENE_KNOT]
  SolveInterpolation(data_pts, t);       // [SHENE_INT]
}

// ---------------------------------------------------------------------------
// ChordLength – Chord-Length-Parametrisierung [SHENE_CHORD].
// Weist jedem Datenpunkt D_k einen normierten Parameter t_k ∈ [0,1] zu,
// proportional zur Bogenlänge des Polygonzugs.
// ---------------------------------------------------------------------------
std::vector<double> BSpline::ChordLength(const std::vector<Point2D>& pts) {
  const int n = static_cast<int>(pts.size());
  std::vector<double> t(n, 0.0);

  for (int i = 1; i < n; ++i) {
    const double dx = pts[i].x - pts[i - 1].x;
    const double dy = pts[i].y - pts[i - 1].y;
    t[i] = t[i - 1] + std::hypot(dx, dy);
  }

  const double total_length = t[n - 1];

  // Sonderfall: alle Punkte identisch → uniforme Parametrisierung.
  if (total_length < 1e-9) {
    for (int i = 0; i < n; ++i) {
      t[i] = static_cast<double>(i) / (n - 1);
    }
    return t;
  }

  for (int i = 0; i < n; ++i) {
    t[i] /= total_length;
  }
  t[n - 1] = 1.0;  // Exakten Endwert erzwingen.
  return t;
}

// ---------------------------------------------------------------------------
// BuildKnotVector – Knotenvektor nach Averaging-Regel [SHENE_KNOT].
// Erzeugt einen Knotenvektor der Länge m = n + p + 1 mit
// p+1-facher Klammerung an beiden Enden.
// ---------------------------------------------------------------------------
void BSpline::BuildKnotVector(
    const std::vector<double>& t, int n, int p) {
  const int m = n + p + 1;
  knots_.assign(m, 0.0);

  // Erste p+1 Knoten = 0, letzte p+1 Knoten = 1.
  for (int i = n; i < m; ++i) {
    knots_[i] = 1.0;
  }

  // Innere Knoten via Averaging-Regel (Shene-Notation: j = 1..n-p-1).
  for (int j = 1; j <= n - p - 1; ++j) {
    double sum = 0.0;
    for (int i = j; i <= j + p - 1; ++i) {
      sum += t[i];
    }
    knots_[j + p] = sum / p;
  }
}

// ---------------------------------------------------------------------------
// SolveInterpolation – Löst N·P = D via QR-Zerlegung [SHENE_INT].
// N(k,j) = N_{j,p}(t_k); P_x, P_y werden unabhängig gelöst.
// ---------------------------------------------------------------------------
void BSpline::SolveInterpolation(
    const std::vector<Point2D>& D, const std::vector<double>& t) {
  const int n = static_cast<int>(D.size());

  Eigen::MatrixXd N(n, n);
  for (int k = 0; k < n; ++k) {
    for (int j = 0; j < n; ++j) {
      N(k, j) = BasisFunction(j, degree_, t[k]);
    }
  }

  Eigen::VectorXd D_x(n), D_y(n);
  for (int i = 0; i < n; ++i) {
    D_x(i) = D[i].x;
    D_y(i) = D[i].y;
  }

  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> solver(N);
  const Eigen::VectorXd P_x = solver.solve(D_x);
  const Eigen::VectorXd P_y = solver.solve(D_y);

  control_points_.clear();
  control_points_.reserve(n);
  for (int i = 0; i < n; ++i) {
    control_points_.push_back({P_x(i), P_y(i)});
  }
}

// ---------------------------------------------------------------------------
// BasisFunction – Cox–de-Boor-Rekursion [SHENE_BASIS].
// Berechnet N_{i,p}(u). Randbedingung u = t_max wird separat behandelt,
// damit der letzte Kontrollpunkt exakt interpoliert wird.
// ---------------------------------------------------------------------------
double BSpline::BasisFunction(int i, int p, double u) const {
  if (p == 0) {
    // Randbedingung: u = 1.0 → letztes nicht-leeres Intervall aktiv.
    if (u >= 1.0 - 1e-10) {
      return (knots_[i] < 1.0 && knots_[i + 1] >= 1.0) ? 1.0 : 0.0;
    }
    return (knots_[i] <= u && u < knots_[i + 1]) ? 1.0 : 0.0;
  }

  double left = 0.0;
  double right = 0.0;

  const double span1 = knots_[i + p]     - knots_[i];
  const double span2 = knots_[i + p + 1] - knots_[i + 1];

  if (std::abs(span1) > 1e-10) {
    left = ((u - knots_[i]) / span1) * BasisFunction(i, p - 1, u);
  }
  if (std::abs(span2) > 1e-10) {
    right = ((knots_[i + p + 1] - u) / span2) * BasisFunction(i + 1, p - 1, u);
  }

  return left + right;
}

// ---------------------------------------------------------------------------
// Evaluate – Wertet C(u) = Σ N_{i,p}(u) · P_i aus.
// ---------------------------------------------------------------------------
Point2D BSpline::Evaluate(double u) const {
  u = std::clamp(u, 0.0, 1.0);

  Point2D result{0.0, 0.0};
  for (int i = 0; i < static_cast<int>(control_points_.size()); ++i) {
    const double b = BasisFunction(i, degree_, u);
    result.x += b * control_points_[i].x;
    result.y += b * control_points_[i].y;
  }
  return result;
}

// ---------------------------------------------------------------------------
// Sample – Gleichmäßige Abtastung mit num_samples Punkten in [0, 1].
// ---------------------------------------------------------------------------
std::vector<Point2D> BSpline::Sample(int num_samples) const {
  std::vector<Point2D> out;
  out.reserve(num_samples);

  for (int i = 0; i < num_samples; ++i) {
    const double u = static_cast<double>(i) / (num_samples - 1);
    out.push_back(Evaluate(u));
  }
  return out;
}

}  // namespace bspline_smoother