#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include <delaunator.hpp>

// ============================================================
// Coaxial pair scattered-data linear interpolator (C++ version)
// Equivalent idea to SciPy LinearNDInterpolator:
//   1) Delaunay triangulation
//   2) linear barycentric interpolation inside each triangle
//   3) nearest-neighbor fallback outside convex hull
// ============================================================

struct InverseResult {
    bool success{false};

    double omega_u{0.0};
    double omega_l{0.0};

    double T_pred{0.0};
    double Q_pred{0.0};

    double objective{0.0};   // normalized residual for fallback
    std::string status;      // "exact_triangle_inverse" / "local_search" / "failed"
};

struct Sample {
    double upper_rpm; // use magnitude only
    double lower_rpm; // use magnitude only
    double T_total;   // N
    double Q_total;   // N*m
};

struct QueryResult {
    double T_total{0.0};
    double Q_total{0.0};
    std::string status; // "linear" or "nearest_fallback"
};

class CoaxialPairInterpolator {
public:
    explicit CoaxialPairInterpolator(const std::vector<Sample>& samples)
        : samples_(samples) {
        if (samples_.size() < 3) {
            throw std::runtime_error("Need at least 3 points for triangulation.");
        }

        coords_.reserve(samples_.size() * 2);

        u_min_ = std::numeric_limits<double>::infinity();
        u_max_ = -std::numeric_limits<double>::infinity();
        l_min_ = std::numeric_limits<double>::infinity();
        l_max_ = -std::numeric_limits<double>::infinity();

        for (const auto& s : samples_) {
            coords_.push_back(s.upper_rpm);
            coords_.push_back(s.lower_rpm);

            u_min_ = std::min(u_min_, s.upper_rpm);
            u_max_ = std::max(u_max_, s.upper_rpm);
            l_min_ = std::min(l_min_, s.lower_rpm);
            l_max_ = std::max(l_max_, s.lower_rpm);
        }

        triangulation_ = std::make_unique<delaunator::Delaunator>(coords_);
    }

    QueryResult query(double omega_u, double omega_l, bool clamp = false) const {
        double wu = std::abs(omega_u);
        double wl = std::abs(omega_l);

        if (clamp) {
            wu = std::clamp(wu, u_min_, u_max_);
            wl = std::clamp(wl, l_min_, l_max_);
        }

        // Search which triangle contains the query point
        const auto& tris = triangulation_->triangles;
        for (std::size_t t = 0; t + 2 < tris.size(); t += 3) {
            std::size_t i = tris[t];
            std::size_t j = tris[t + 1];
            std::size_t k = tris[t + 2];

            double l1, l2, l3;
            if (barycentric(wu, wl, samples_[i], samples_[j], samples_[k], l1, l2, l3)) {
                QueryResult out;
                out.T_total = l1 * samples_[i].T_total + l2 * samples_[j].T_total + l3 * samples_[k].T_total;
                out.Q_total = l1 * samples_[i].Q_total + l2 * samples_[j].Q_total + l3 * samples_[k].Q_total;
                out.status = "linear";
                return out;
            }
        }

        // Outside convex hull -> nearest fallback
        std::size_t idx = nearestNeighbor(wu, wl);
        QueryResult out;
        out.T_total = samples_[idx].T_total;
        out.Q_total = samples_[idx].Q_total;
        out.status = "nearest_fallback";
        return out;
    }

    void printBoundingBox() const {
        std::cout << "RPM bounding box:\n";
        std::cout << "  upper: [" << u_min_ << ", " << u_max_ << "]\n";
        std::cout << "  lower: [" << l_min_ << ", " << l_max_ << "]\n";
    }

public:
    InverseResult invert(double T_des, double Q_des, bool use_fallback_search = true) const;

private:
    static bool solve2x2(double a11, double a12,
                         double a21, double a22,
                         double b1,  double b2,
                         double& x1, double& x2)
    {
        const double det = a11 * a22 - a12 * a21;
        constexpr double eps = 1e-12;
        if (std::abs(det) < eps) {
            return false;
        }

        x1 = ( b1 * a22 - b2 * a12) / det;
        x2 = ( a11 * b2 - a21 * b1) / det;
        return true;
    };

    static double sqr(double x){
    return x * x;
    };

    static bool barycentricFromOutputTriangle(
        double T_des, double Q_des,
        const Sample& s1, const Sample& s2, const Sample& s3,
        double& l1, double& l2, double& l3);

    std::size_t nearestSampleInTQ(double T_des, double Q_des) const;

    double normalizedObjective(double T_pred, double Q_pred,
                               double T_des, double Q_des) const;

    InverseResult localSearch(double T_des, double Q_des,
                              double init_u, double init_l) const;

    static bool barycentric(double x, double y,
                            const Sample& a,
                            const Sample& b,
                            const Sample& c,
                            double& l1, double& l2, double& l3) {
        const double x1 = a.upper_rpm, y1 = a.lower_rpm;
        const double x2 = b.upper_rpm, y2 = b.lower_rpm;
        const double x3 = c.upper_rpm, y3 = c.lower_rpm;

        const double den =
            (y2 - y3) * (x1 - x3) +
            (x3 - x2) * (y1 - y3);

        constexpr double eps = 1e-12;
        if (std::abs(den) < eps) {
            return false; // degenerate triangle
        }

        l1 = ((y2 - y3) * (x - x3) + (x3 - x2) * (y - y3)) / den;
        l2 = ((y3 - y1) * (x - x3) + (x1 - x3) * (y - y3)) / den;
        l3 = 1.0 - l1 - l2;

        // allow tiny negative due to floating-point
        constexpr double tol = -1e-10;
        return (l1 >= tol && l2 >= tol && l3 >= tol);
    }

    std::size_t nearestNeighbor(double x, double y) const {
        std::size_t best_idx = 0;
        double best_d2 = std::numeric_limits<double>::infinity();

        for (std::size_t i = 0; i < samples_.size(); ++i) {
            const double dx = x - samples_[i].upper_rpm;
            const double dy = y - samples_[i].lower_rpm;
            const double d2 = dx * dx + dy * dy;
            if (d2 < best_d2) {
                best_d2 = d2;
                best_idx = i;
            }
        }
        return best_idx;
    }

private:
    std::vector<Sample> samples_;
    std::vector<double> coords_;
    std::unique_ptr<delaunator::Delaunator> triangulation_;

    double u_min_{0.0}, u_max_{0.0};
    double l_min_{0.0}, l_max_{0.0};
};

inline bool CoaxialPairInterpolator::barycentricFromOutputTriangle(
    double T_des, double Q_des,
    const Sample& s1, const Sample& s2, const Sample& s3,
    double& l1, double& l2, double& l3) {

    // Solve in output space:
    // [T1-T3  T2-T3] [l1] = [Tdes-T3]
    // [Q1-Q3  Q2-Q3] [l2]   [Qdes-Q3]
    double x1 = 0.0, x2 = 0.0;
    bool ok = solve2x2(
        s1.T_total - s3.T_total, s2.T_total - s3.T_total,
        s1.Q_total - s3.Q_total, s2.Q_total - s3.Q_total,
        T_des - s3.T_total,      Q_des - s3.Q_total,
        x1, x2
    );

    if (!ok) {
        return false;
    }

    l1 = x1;
    l2 = x2;
    l3 = 1.0 - l1 - l2;

    constexpr double tol = -1e-10;
    return (l1 >= tol && l2 >= tol && l3 >= tol);
}

inline std::size_t CoaxialPairInterpolator::nearestSampleInTQ(double T_des, double Q_des) const {
    std::size_t best_idx = 0;
    double best_obj = std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < samples_.size(); ++i) {
        const double obj = normalizedObjective(
            samples_[i].T_total, samples_[i].Q_total,
            T_des, Q_des
        );
        if (obj < best_obj) {
            best_obj = obj;
            best_idx = i;
        }
    }
    return best_idx;
}

inline double CoaxialPairInterpolator::normalizedObjective(double T_pred, double Q_pred,
                                                           double T_des, double Q_des) const {
    const double T_scale = std::max(1.0, std::abs(T_des));
    const double Q_scale = std::max(1.0, std::abs(Q_des));

    const double eT = (T_pred - T_des) / T_scale;
    const double eQ = (Q_pred - Q_des) / Q_scale;

    return sqr(eT) + sqr(eQ);
}

inline InverseResult CoaxialPairInterpolator::localSearch(double T_des, double Q_des,
                                                          double init_u, double init_l) const {
    InverseResult best;
    best.success = true;
    best.omega_u = std::clamp(init_u, u_min_, u_max_);
    best.omega_l = std::clamp(init_l, l_min_, l_max_);

    {
        auto qr = query(best.omega_u, best.omega_l, true);
        best.T_pred = qr.T_total;
        best.Q_pred = qr.Q_total;
        best.objective = normalizedObjective(best.T_pred, best.Q_pred, T_des, Q_des);
        best.status = "local_search";
    }

    double step_u = std::max(50.0, 0.2 * (u_max_ - u_min_));
    double step_l = std::max(50.0, 0.2 * (l_max_ - l_min_));

    for (int outer = 0; outer < 20; ++outer) {
        bool improved = false;

        for (int du_i = -1; du_i <= 1; ++du_i) {
            for (int dl_i = -1; dl_i <= 1; ++dl_i) {
                const double cand_u = std::clamp(best.omega_u + du_i * step_u, u_min_, u_max_);
                const double cand_l = std::clamp(best.omega_l + dl_i * step_l, l_min_, l_max_);

                auto qr = query(cand_u, cand_l, true);
                const double obj = normalizedObjective(qr.T_total, qr.Q_total, T_des, Q_des);

                if (obj < best.objective) {
                    best.omega_u = cand_u;
                    best.omega_l = cand_l;
                    best.T_pred = qr.T_total;
                    best.Q_pred = qr.Q_total;
                    best.objective = obj;
                    improved = true;
                }
            }
        }

        if (!improved) {
            step_u *= 0.5;
            step_l *= 0.5;
        }

        if (step_u < 1.0 && step_l < 1.0) {
            break;
        }
    }

    return best;
}

inline InverseResult CoaxialPairInterpolator::invert(double T_des, double Q_des,
                                                     bool use_fallback_search) const {
    const auto& tris = triangulation_->triangles;

    // ------------------------------------------------------------
    // 1) Exact inverse on each Delaunay triangle in output space
    // ------------------------------------------------------------
    for (std::size_t t = 0; t + 2 < tris.size(); t += 3) {
        const std::size_t i = tris[t];
        const std::size_t j = tris[t + 1];
        const std::size_t k = tris[t + 2];

        double l1 = 0.0, l2 = 0.0, l3 = 0.0;
        if (!barycentricFromOutputTriangle(
                T_des, Q_des,
                samples_[i], samples_[j], samples_[k],
                l1, l2, l3)) {
            continue;
        }

        InverseResult out;
        out.success = true;
        out.omega_u = l1 * samples_[i].upper_rpm +
                      l2 * samples_[j].upper_rpm +
                      l3 * samples_[k].upper_rpm;

        out.omega_l = l1 * samples_[i].lower_rpm +
                      l2 * samples_[j].lower_rpm +
                      l3 * samples_[k].lower_rpm;

        auto qr = query(out.omega_u, out.omega_l, true);
        out.T_pred = qr.T_total;
        out.Q_pred = qr.Q_total;
        out.objective = normalizedObjective(out.T_pred, out.Q_pred, T_des, Q_des);
        out.status = "exact_triangle_inverse";
        return out;
    }

    // ------------------------------------------------------------
    // 2) Fallback: bounded derivative-free local search
    // ------------------------------------------------------------
    if (use_fallback_search) {
        const std::size_t idx0 = nearestSampleInTQ(T_des, Q_des);
        return localSearch(T_des, Q_des,
                           samples_[idx0].upper_rpm,
                           samples_[idx0].lower_rpm);
    }

    InverseResult fail;
    fail.success = false;
    fail.status = "failed";
    return fail;
}