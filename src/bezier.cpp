#include "bezier.hpp"
#include "types.hpp"
#include <cmath>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <limits>
#include <sstream>

// Compute derivative of a cubic Bezier curve
Point bezierDerivative(const std::vector<Point>& controlPoints, float t) {
	float dx = 3 * (1 - t) * (1 - t) * (controlPoints[1].x - controlPoints[0].x) +
		6 * (1 - t) * t * (controlPoints[2].x - controlPoints[1].x) +
		3 * t * t * (controlPoints[3].x - controlPoints[2].x);
	float dy = 3 * (1 - t) * (1 - t) * (controlPoints[1].y - controlPoints[0].y) +
		6 * (1 - t) * t * (controlPoints[2].y - controlPoints[1].y) +
		3 * t * t * (controlPoints[3].y - controlPoints[2].y);
	return {dx, dy};
}
// Compute second derivative of a cubic Bezier curve
Point bezierSecondDerivative(const std::vector<Point>& controlPoints, float t) {
	float dx = 6 * (1 - t) * (controlPoints[2].x - 2 * controlPoints[1].x + controlPoints[0].x) +
		6 * t * (controlPoints[3].x - 2 * controlPoints[2].x + controlPoints[1].x);
	float dy = 6 * (1 - t) * (controlPoints[2].y - 2 * controlPoints[1].y + controlPoints[0].y) +
		6 * t * (controlPoints[3].y - 2 * controlPoints[2].y + controlPoints[1].y);
	return {dx, dy};
}
// Compute speed function ||r'(t)||
float speed(const std::vector<Point>& controlPoints, float t) {
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
float arcLength(const std::vector<Point>& controlPoints, float a, float b) {
	const double half = (b - a) / 2.0;
	const double mid = (a + b) / 2.0;
	double length = 0.0;
	for (int i = 0; i < kGaussN; i++) {
		const double t = half * kGaussNodes[i] + mid;
		length += kGaussWeights[i] * speed(controlPoints, static_cast<float>(t));
	}
	return static_cast<float>(half * length);
}

// Compute s(t) from 0 to t using composite quadrature.
float sFunction(const std::vector<Point>& controlPoints, float t) {
	if (t <= 0.0f) {
		return 0.0f;
	}
	const double step = static_cast<double>(t) / kArcLengthPanels;
	double total = 0.0;
	for (int i = 0; i < kArcLengthPanels; i++) {
		total += arcLength(controlPoints, static_cast<float>(step * i),
		                   static_cast<float>(step * (i + 1)));
	}
	return static_cast<float>(total);
}

// Solve s(t) = sCurrent + deltaS for t by Newton-Raphson, falling back to
// bisection when the parametric speed is too small for Newton to be stable.
float findTForS(const std::vector<Point>& controlPoints, float sCurrent, float deltaS,
                float tGuess) {
	const float target = sCurrent + deltaS;
	const float tol = 1e-6f;
	const int maxIter = 20;

	// s(t) is monotonically increasing, so a target past the end saturates.
	if (sFunction(controlPoints, 1.0f) <= target) {
		return 1.0f;
	}

	float t = std::clamp(tGuess, 0.0f, 1.0f);
	for (int i = 0; i < maxIter; i++) {
		const float residual = sFunction(controlPoints, t) - target;
		if (std::fabs(residual) < tol) {
			return t;
		}

		const float derivative = speed(controlPoints, t);
		if (derivative < 1e-9f) {
			break; // Near-cusp: Newton is unusable here.
		}

		const float tNext = std::clamp(t - residual / derivative, 0.0f, 1.0f);
		if (std::fabs(tNext - t) < tol) {
			return tNext;
		}
		t = tNext;
	}

	float lo = 0.0f;
	float hi = 1.0f;
	for (int i = 0; i < 50; i++) {
		const float mid = 0.5f * (lo + hi);
		if (sFunction(controlPoints, mid) < target) {
			lo = mid;
		} else {
			hi = mid;
		}
	}
	return 0.5f * (lo + hi);
}

static float distanceSquared(const std::vector<Point>& controlPoints, float t, float x, float y) {
	const Pose p = findXandY(controlPoints, t);
	const float dx = p.x - x;
	const float dy = p.y - y;
	return dx * dx + dy * dy;
}

// Seeds for the projection refinement. The squared-distance function is not
// convex, so a coarse scan is needed to land in the right basin first.
static constexpr int kProjectionSamples = 64;

float projectOntoCurve(const std::vector<Point>& controlPoints, float x, float y,
                       float* residual) {
	float bestT = 0.0f;
	float bestDistSq = distanceSquared(controlPoints, 0.0f, x, y);
	for (int i = 1; i <= kProjectionSamples; i++) {
		const float t = static_cast<float>(i) / kProjectionSamples;
		const float distSq = distanceSquared(controlPoints, t, x, y);
		if (distSq < bestDistSq) {
			bestDistSq = distSq;
			bestT = t;
		}
	}

	// Refine with Newton on d/dt ||r(t) - p||^2 = 2 (r(t) - p) . r'(t) = 0.
	float t = bestT;
	for (int i = 0; i < 20; i++) {
		const Pose pos = findXandY(controlPoints, t);
		const Point r1 = bezierDerivative(controlPoints, t);
		const Point r2 = bezierSecondDerivative(controlPoints, t);
		const float ex = pos.x - x;
		const float ey = pos.y - y;
		const float f = ex * r1.x + ey * r1.y;
		const float fPrime = r1.x * r1.x + r1.y * r1.y + ex * r2.x + ey * r2.y;
		if (std::fabs(fPrime) < 1e-12f) {
			break;
		}
		const float tNext = std::clamp(t - f / fPrime, 0.0f, 1.0f);
		const bool converged = std::fabs(tNext - t) < 1e-6f;
		t = tNext;
		if (converged) {
			break;
		}
	}

	// Newton can walk uphill on a non-convex objective; keep the better root.
	const float refinedDistSq = distanceSquared(controlPoints, t, x, y);
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
static constexpr float kKeyframeOffPathTolerance = 0.05f;

std::vector<KeyframeVelocities> convertToTFrame(
	const std::vector<Point>& bezierPoints,
	const std::vector<KeyframeVelocitiesXandY>& keyFrameVelocitiesXY
) {
	std::vector<KeyframeVelocities> keyFrameVelocitiesT;
	keyFrameVelocitiesT.reserve(keyFrameVelocitiesXY.size());
	float prevT = 0.0f;

	for (const auto& kf : keyFrameVelocitiesXY) {
		float residual = 0.0f;
		const float t = projectOntoCurve(bezierPoints, kf.x, kf.y, &residual);

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
static constexpr float kDegenerateCurvature = 1.0e4f;

// Curvature magnitude and sign share a denominator; compute once.
static float curvatureImpl(const std::vector<Point>& controlPoints, float t, bool signedResult) {
	const Point r1 = bezierDerivative(controlPoints, t);
	const Point r2 = bezierSecondDerivative(controlPoints, t);

	// 2D cross product (determinant form)
	const float crossProduct = r1.x * r2.y - r1.y * r2.x;

	// |r'(t)|^3, computed without a pow() round trip
	const float speedSquared = r1.x * r1.x + r1.y * r1.y;
	const float speedCubed = speedSquared * std::sqrt(speedSquared);

	if (speedCubed < 1e-12f) {
		if (!signedResult) {
			return kDegenerateCurvature;
		}
		// Preserve turn direction where it is still recoverable.
		return crossProduct < 0.0f ? -kDegenerateCurvature : kDegenerateCurvature;
	}

	const float kappa = crossProduct / speedCubed;
	const float clamped = std::clamp(kappa, -kDegenerateCurvature, kDegenerateCurvature);
	return signedResult ? clamped : std::fabs(clamped);
}

// Compute signed curvature k(t)
float signedCurvature(const std::vector<Point>& controlPoints, float t) {
	return curvatureImpl(controlPoints, t, true);
}

// Compute unsigned curvature |k(t)|
float unsignedCurvature(const std::vector<Point>& controlPoints, float t) {
	return curvatureImpl(controlPoints, t, false);
}

Pose findXandY(const std::vector<Point>& controlPoints, float t) {
	float x = std::pow(1 - t, 3) * controlPoints[0].x +
		3 * std::pow(1 - t, 2) * t * controlPoints[1].x +
		3 * (1 - t) * std::pow(t, 2) * controlPoints[2].x +
		std::pow(t, 3) * controlPoints[3].x;

	float y = std::pow(1 - t, 3) * controlPoints[0].y +
		3 * std::pow(1 - t, 2) * t * controlPoints[1].y +
		3 * (1 - t) * std::pow(t, 2) * controlPoints[2].y +
		std::pow(t, 3) * controlPoints[3].y;
	Point derivative = bezierDerivative(controlPoints, t);
	float theta = std::atan2(derivative.y, derivative.x);
	return {x, y, theta};
}
