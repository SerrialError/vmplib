#include "bezier.hpp"
#include "types.hpp"
#include <cmath>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <limits>
#include <sstream>

// A cubic Bezier segment is exactly four control points; anything else would
// make the bezier* routines index past the end of the vector.
static constexpr size_t kCubicControlPoints = 4;

void requireCubicSegment(const std::vector<Point>& controlPoints) {
	if (controlPoints.size() != kCubicControlPoints) {
		std::ostringstream msg;
		msg << "cubic Bezier segment needs exactly " << kCubicControlPoints
		    << " control points, got " << controlPoints.size();
		throw SegmentError(msg.str());
	}
}

// Compute derivative of a cubic Bezier curve
Point bezierDerivative(const std::vector<Point>& controlPoints, double t) {
	double dx = 3 * (1 - t) * (1 - t) * (controlPoints[1].x - controlPoints[0].x) +
		6 * (1 - t) * t * (controlPoints[2].x - controlPoints[1].x) +
		3 * t * t * (controlPoints[3].x - controlPoints[2].x);
	double dy = 3 * (1 - t) * (1 - t) * (controlPoints[1].y - controlPoints[0].y) +
		6 * (1 - t) * t * (controlPoints[2].y - controlPoints[1].y) +
		3 * t * t * (controlPoints[3].y - controlPoints[2].y);
	return {dx, dy};
}
// Compute second derivative of a cubic Bezier curve
Point bezierSecondDerivative(const std::vector<Point>& controlPoints, double t) {
	double dx = 6 * (1 - t) * (controlPoints[2].x - 2 * controlPoints[1].x + controlPoints[0].x) +
		6 * t * (controlPoints[3].x - 2 * controlPoints[2].x + controlPoints[1].x);
	double dy = 6 * (1 - t) * (controlPoints[2].y - 2 * controlPoints[1].y + controlPoints[0].y) +
		6 * t * (controlPoints[3].y - 2 * controlPoints[2].y + controlPoints[1].y);
	return {dx, dy};
}
// Compute speed function ||r'(t)||
double speed(const std::vector<Point>& controlPoints, double t) {
	Point deriv = bezierDerivative(controlPoints, t);
	return std::sqrt(deriv.x * deriv.x + deriv.y * deriv.y);
}
// 5-point Gauss-Legendre nodes and weights on [-1, 1].
static constexpr int kGaussN = 5;
static constexpr double kGaussNodes[kGaussN] = {
	-0.9061798459386640, -0.5384693101056831, 0.0,
	0.5384693101056831, 0.9061798459386640};
static constexpr double kGaussWeights[kGaussN] = {
	0.2369268850561891, 0.4786286704993665, 0.5688888888888889,
	0.4786286704993665, 0.2369268850561891};

// Number of panels used by sFunction. The integrand is the square root of a
// polynomial rather than a polynomial, so the degree-(2n-1) exactness of
// Gauss-Legendre does not apply and a single panel over [0, t] leaves real
// error on long or wiggly curves. Subdividing restores accuracy cheaply.
static constexpr int kArcLengthPanels = 8;

// Compute arc length over [a, b] with a single Gaussian quadrature panel.
double arcLength(const std::vector<Point>& controlPoints, double a, double b) {
	const double half = (b - a) / 2.0;
	const double mid = (a + b) / 2.0;
	double length = 0.0;
	for (int i = 0; i < kGaussN; i++) {
		const double t = half * kGaussNodes[i] + mid;
		length += kGaussWeights[i] * speed(controlPoints, t);
	}
	return half * length;
}

// Compute s(t) from 0 to t using composite quadrature.
double sFunction(const std::vector<Point>& controlPoints, double t) {
	if (t <= 0.0) {
		return 0.0;
	}
	const double step = t / kArcLengthPanels;
	double total = 0.0;
	for (int i = 0; i < kArcLengthPanels; i++) {
		total += arcLength(controlPoints, step * i, step * (i + 1));
	}
	return total;
}

// Solve s(t) = sCurrent + deltaS for t by Newton-Raphson, falling back to
// bisection when the parametric speed is too small for Newton to be stable.
double findTForS(const std::vector<Point>& controlPoints, double sCurrent, double deltaS,
                 double tGuess) {
	const double target = sCurrent + deltaS;
	const double tol = 1e-9;
	const int maxIter = 20;

	// s(t) is monotonically increasing, so a target past the end saturates.
	if (sFunction(controlPoints, 1.0) <= target) {
		return 1.0;
	}

	double t = std::clamp(tGuess, 0.0, 1.0);
	for (int i = 0; i < maxIter; i++) {
		const double residual = sFunction(controlPoints, t) - target;
		if (std::fabs(residual) < tol) {
			return t;
		}

		const double derivative = speed(controlPoints, t);
		if (derivative < 1e-12) {
			break; // Near-cusp: Newton is unusable here.
		}

		const double tNext = std::clamp(t - residual / derivative, 0.0, 1.0);
		if (std::fabs(tNext - t) < tol) {
			return tNext;
		}
		t = tNext;
	}

	double lo = 0.0;
	double hi = 1.0;
	for (int i = 0; i < 50; i++) {
		const double mid = 0.5 * (lo + hi);
		if (sFunction(controlPoints, mid) < target) {
			lo = mid;
		} else {
			hi = mid;
		}
	}
	return 0.5 * (lo + hi);
}

static double distanceSquared(const std::vector<Point>& controlPoints, double t, double x, double y) {
	const Pose p = findXandY(controlPoints, t);
	const double dx = p.x - x;
	const double dy = p.y - y;
	return dx * dx + dy * dy;
}

// Seeds for the projection refinement. The squared-distance function is not
// convex, so a coarse scan is needed to land in the right basin first.
static constexpr int kProjectionSamples = 64;

double projectOntoCurve(const std::vector<Point>& controlPoints, double x, double y,
                        double* residual) {
	double bestT = 0.0;
	double bestDistSq = distanceSquared(controlPoints, 0.0, x, y);
	for (int i = 1; i <= kProjectionSamples; i++) {
		const double t = static_cast<double>(i) / kProjectionSamples;
		const double distSq = distanceSquared(controlPoints, t, x, y);
		if (distSq < bestDistSq) {
			bestDistSq = distSq;
			bestT = t;
		}
	}

	// Refine with Newton on d/dt ||r(t) - p||^2 = 2 (r(t) - p) . r'(t) = 0.
	double t = bestT;
	for (int i = 0; i < 20; i++) {
		const Pose pos = findXandY(controlPoints, t);
		const Point r1 = bezierDerivative(controlPoints, t);
		const Point r2 = bezierSecondDerivative(controlPoints, t);
		const double ex = pos.x - x;
		const double ey = pos.y - y;
		const double f = ex * r1.x + ey * r1.y;
		const double fPrime = r1.x * r1.x + r1.y * r1.y + ex * r2.x + ey * r2.y;
		if (std::fabs(fPrime) < 1e-15) {
			break;
		}
		const double tNext = std::clamp(t - f / fPrime, 0.0, 1.0);
		const bool converged = std::fabs(tNext - t) < 1e-9;
		t = tNext;
		if (converged) {
			break;
		}
	}

	// Newton can walk uphill on a non-convex objective; keep the better root.
	const double refinedDistSq = distanceSquared(controlPoints, t, x, y);
	if (refinedDistSq > bestDistSq) {
		t = bestT;
	}
	if (residual != nullptr) {
		*residual = std::sqrt(distanceSquared(controlPoints, t, x, y));
	}
	return t;
}

// A keyframe further than this from the curve is treated as a path-authoring
// mistake rather than a rounding artifact. Metres, i.e. field units.
static constexpr double kKeyframeOffPathTolerance = 0.05;

std::vector<KeyframeVelocities> convertToTFrame(
	const std::vector<Point>& bezierPoints,
	const std::vector<KeyframeVelocitiesXandY>& keyFrameVelocitiesXY
) {
	requireCubicSegment(bezierPoints);

	std::vector<KeyframeVelocities> keyFrameVelocitiesT;
	keyFrameVelocitiesT.reserve(keyFrameVelocitiesXY.size());
	double prevT = 0.0;

	for (const auto& kf : keyFrameVelocitiesXY) {
		double residual = 0.0;
		const double t = projectOntoCurve(bezierPoints, kf.x, kf.y, &residual);

		if (residual > kKeyframeOffPathTolerance) {
			std::ostringstream msg;
			msg << "keyframe at (" << kf.x << ", " << kf.y << ") is " << residual
			    << " off the path (tolerance " << kKeyframeOffPathTolerance
			    << "); its velocity would be applied at t=" << t;
			throw KeyframeError(msg.str());
		}
		// computeKeyframeLimit walks a sorted list, so out-of-order keyframes
		// would silently bracket the wrong interval.
		if (t < prevT) {
			std::ostringstream msg;
			msg << "keyframe at (" << kf.x << ", " << kf.y << ") projects to t=" << t
			    << ", behind the previous keyframe at t=" << prevT
			    << "; keyframes must advance along the path";
			throw KeyframeError(msg.str());
		}

		keyFrameVelocitiesT.push_back({kf.velocity, t});
		prevT = t;
	}
	return keyFrameVelocitiesT;
}
// Curvature returned where |r'(t)| collapses. A cusp has infinite curvature,
// so saturating high is the safe direction: the curvature speed limit
// v_max * R/(R + w/2) then drives the commanded speed toward zero. Returning
// zero instead would remove the speed limit at the sharpest point on the path.
// The resulting angular velocity tends to 2*v_max/w, i.e. turning in place at
// the drivetrain's maximum rate, which is the correct physical limit.
static constexpr double kDegenerateCurvature = 1.0e4;

// Curvature magnitude and sign share a denominator; compute once.
static double curvatureImpl(const std::vector<Point>& controlPoints, double t, bool signedResult) {
	const Point r1 = bezierDerivative(controlPoints, t);
	const Point r2 = bezierSecondDerivative(controlPoints, t);

	// 2D cross product (determinant form)
	const double crossProduct = r1.x * r2.y - r1.y * r2.x;

	// |r'(t)|^3, computed without a pow() round trip
	const double speedSquared = r1.x * r1.x + r1.y * r1.y;
	const double speedCubed = speedSquared * std::sqrt(speedSquared);

	if (speedCubed < 1e-15) {
		if (!signedResult) {
			return kDegenerateCurvature;
		}
		// Preserve turn direction where it is still recoverable.
		return crossProduct < 0.0 ? -kDegenerateCurvature : kDegenerateCurvature;
	}

	const double kappa = crossProduct / speedCubed;
	const double clamped = std::clamp(kappa, -kDegenerateCurvature, kDegenerateCurvature);
	return signedResult ? clamped : std::fabs(clamped);
}

// Compute signed curvature k(t)
double signedCurvature(const std::vector<Point>& controlPoints, double t) {
	return curvatureImpl(controlPoints, t, true);
}

// Compute unsigned curvature |k(t)|
double unsignedCurvature(const std::vector<Point>& controlPoints, double t) {
	return curvatureImpl(controlPoints, t, false);
}

Pose findXandY(const std::vector<Point>& controlPoints, double t) {
	double x = std::pow(1 - t, 3) * controlPoints[0].x +
		3 * std::pow(1 - t, 2) * t * controlPoints[1].x +
		3 * (1 - t) * std::pow(t, 2) * controlPoints[2].x +
		std::pow(t, 3) * controlPoints[3].x;

	double y = std::pow(1 - t, 3) * controlPoints[0].y +
		3 * std::pow(1 - t, 2) * t * controlPoints[1].y +
		3 * (1 - t) * std::pow(t, 2) * controlPoints[2].y +
		std::pow(t, 3) * controlPoints[3].y;
	Point derivative = bezierDerivative(controlPoints, t);
	double theta = std::atan2(derivative.y, derivative.x);
	return {x, y, theta};
}
