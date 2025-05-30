#include "bezier.hpp"
#include <cmath>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <limits>

float sCurrent = -100;
float prev_t = 0;
float tim = 0;
float curSpeed = 0.001;
float prevKeyFrame = 0;

// Vector to store the points
std::vector<Pose> poses;
std::vector<Point> pose;
std::vector<Pose> ramsetePoses;

// Vector to store the points
std::vector<VelocityLayout> velocities;
std::vector<VelocityLayout> dec_velocities;
std::vector<VelocityLayout> ramseteVelocities;
static float fsign(float x) {
  return static_cast<float>((x > 0) - (x < 0));
}
FirstOrderWithDeadBand::FirstOrderWithDeadBand(float K, float tau, float u_static) : K(K), tau(tau), u_static(u_static), y(0), dy(0) {}

float FirstOrderWithDeadBand::update(float u, float dt) {
    dy = (fmaxf(fabsf(K * (u - u_static)), 0.f) * fsign(u) - y) / tau;
    y += dy * dt;
    return y;
}

void FirstOrderWithDeadBand::reset() {
    y = 0;
}

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
// Compute arc length using Gaussian quadrature
float arcLength(const std::vector<Point>& controlPoints, float a, float b) {
	// Gaussian quadrature weights and nodes for n=5, if using quadratic bezier n=3
	const std::vector<float> gaussNodes = {-0.9061798459, -0.5384693101, 0.0, 0.5384693101, 0.9061798459};
	const std::vector<float> gaussWeights = {0.2369268850, 0.4786286705, 0.5688888889, 0.4786286705, 0.2369268850};
	float length = 0.0;
	for (size_t i = 0; i < gaussNodes.size(); i++) {
		float t = (b - a) / 2.0 * gaussNodes[i] + (a + b) / 2.0;
		length += gaussWeights[i] * speed(controlPoints, t);
	}
	return (b - a) / 2.0 * length;
}
// Compute s(t) from 0 to t
float sFunction(const std::vector<Point>& controlPoints, float t) {
	return arcLength(controlPoints, 0, t);
}
// Newton-Raphson to find t for s(t) = s_current + delta_s
float findTForS(const std::vector<Point>& controlPoints, float sCurrent, float deltaS) {
	float tol = 1e-6;
	float t = 0.5; // Initial guess
	int maxIter = 20;
	for (int i = 0; i < maxIter; i++) {
		float s_t = sFunction(controlPoints, t);
		float f_t = s_t - sCurrent - deltaS;
		float f_prime_t = speed(controlPoints, t);

		if (std::fabs(f_t) < tol) break;
		t -= f_t / f_prime_t;
		if (t < 0) t = 0;
		if (t > 1) t = 1;
	}
	return t;
}

// Evaluate cubic Bézier component at t
float bezierComponent(const std::vector<Point>& controlPoints, float t, bool useX) {
    float u = 1 - t;
    float c0 = useX ? controlPoints[0].x : controlPoints[0].y;
    float c1 = useX ? controlPoints[1].x : controlPoints[1].y;
    float c2 = useX ? controlPoints[2].x : controlPoints[2].y;
    float c3 = useX ? controlPoints[3].x : controlPoints[3].y;

    return u*u*u * c0 +
           3*u*u*t * c1 +
           3*u*t*t * c2 +
           t*t*t * c3;
}

// Try Newton-Raphson starting from tGuess
bool tryNewtonRaphson(const std::vector<Point>& controlPoints, float target, bool useX, float& t) {
    float tol = 1e-6f;
    int maxIter = 20;

    for (int i = 0; i < maxIter; ++i) {
        float val = bezierComponent(controlPoints, t, useX);
        Point deriv = bezierDerivative(controlPoints, t);
        float d = useX ? deriv.x : deriv.y;
        float f = val - target;

        if (std::fabs(f) < tol) return true;
        if (std::fabs(d) < 1e-10f) return false;

        t -= f / d;
        if (t < 0.0f || t > 1.0f) return false; // Stay in bounds
    }
    return true;
}

// Main function: find t for x(t) = xTarget or y(t) = yTarget
float findTForComponent(
    const std::vector<Point>& controlPoints,
    float target,          // xTarget or yTarget
    bool useX,             // true for x(t), false for y(t)
    float prevT     // for disambiguating multiple solutions
) {
    std::vector<float> candidates;

    // Try Newton-Raphson from several seeds
    for (float seed : {0.2f, 0.5f, 0.8f}) {
        float t = seed;
        if (tryNewtonRaphson(controlPoints, target, useX, t)) {
            // Avoid duplicates within tolerance
            bool exists = false;
            for (float existing : candidates) {
                if (std::fabs(existing - t) < 1e-4f) {
                    exists = true;
                    break;
                }
            }
            if (!exists) candidates.push_back(t);
        }
    }

    // Fallback: sample curve if Newton-Raphson fails
    if (candidates.empty()) {
        int samples = 100;
        float bestT = 0.0f;
        float minDiff = 1e10f;
        for (int i = 0; i <= samples; ++i) {
            float t = i / float(samples);
            float val = bezierComponent(controlPoints, t, useX);
            float diff = std::fabs(val - target);
            if (diff < minDiff) {
                minDiff = diff;
                bestT = t;
            }
        }
        candidates.push_back(bestT);
    }

    // Return the t closest to prevT
    auto closest = std::min_element(candidates.begin(), candidates.end(),
        [prevT](float a, float b) {
            return std::fabs(a - prevT) < std::fabs(b - prevT);
        });
    
    return *closest;
}

std::vector<KeyframeVelocities> convertToTFrame(
	const std::vector<Point>& bezierPoints,
	const std::vector<KeyframeVelocitiesXandY>& keyFrameVelocitiesXY
) {
	std::vector<KeyframeVelocities> keyFrameVelocitiesT;
	float prevT = 0.0f;

	for (const auto& kf : keyFrameVelocitiesXY) {
        float t = findTForComponent(bezierPoints, kf.x, true, prevT);

		keyFrameVelocitiesT.push_back({kf.velocity, t});
		prevT = t; // update for next iteration
	}
	return keyFrameVelocitiesT;
}
// Compute curvature Îº(t)
float curvature(const std::vector<Point>& controlPoints, float t) {
	Point r1 = bezierDerivative(controlPoints, t);
	Point r2 = bezierSecondDerivative(controlPoints, t);

	// Compute cross product magnitude for 2D case (determinant form)
	float crossProduct = (r1.x * r2.y - r1.y * r2.x);

	// Compute denominator |r'(t)|^3
	float speedCubed = std::pow(std::sqrt(r1.x * r1.x + r1.y * r1.y), 3);

	// Avoid division by zero
	if (speedCubed < 1e-6) return 0.0;

	return crossProduct / speedCubed;
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


Velocities twoDimensionalTrapVel(const std::vector<Point>& controlPoints, float max_linear_velocity, float max_linear_acceleration, float deceleration_distance, float initial_velocity, float exit_velocity, std::vector<KeyframeVelocities>& desired_vels, bool useKeyFrames, float timestep) {
	tim += timestep;
	if (desired_vels[prevKeyFrame + 1].time < prev_t) {
		prevKeyFrame += 1;
	}
	float trackWidth = 0.288925;
	float acceleration_distance = (pow(max_linear_velocity, 2) - pow(initial_velocity, 2)) / (2 * max_linear_acceleration);
	sCurrent = sFunction(controlPoints, prev_t);
	float keyFrameVelocityLimit = 1000;
	if (useKeyFrames) {
	    float v_i2 = desired_vels[prevKeyFrame].velocity * desired_vels[prevKeyFrame].velocity;
    	float v_f2 = desired_vels[prevKeyFrame + 1].velocity * desired_vels[prevKeyFrame + 1].velocity;
	    keyFrameVelocityLimit = sqrt(v_i2 + (v_f2 - v_i2) * ((sFunction(controlPoints, prev_t)) / (sFunction(controlPoints, desired_vels[prevKeyFrame + 1].time))));	    
	}
	// calculates the turning radius for the curvature velocity limit
	float turn_radius = 1 / curvature(controlPoints, prev_t);
	// calculates the curvature velocity limit
	float curvature_velocity_limit = max_linear_velocity * turn_radius / (turn_radius + trackWidth / 2);
	// calculates the acceleration limits
	float linear_velocity_acceleration_limit;
	if (sCurrent < deceleration_distance) {
		linear_velocity_acceleration_limit = curSpeed + (max_linear_acceleration * timestep);

	}
	else {
		if (sCurrent > acceleration_distance) {
			linear_velocity_acceleration_limit = sqrt(pow(exit_velocity, 2) - ((sFunction(controlPoints, 1) - sCurrent) * (pow(exit_velocity, 2) - pow(max_linear_velocity, 2)) / (sFunction(controlPoints, 1) - deceleration_distance)));
		}
	}
	float linear_velocity_deceleration_limit = curSpeed - (max_linear_acceleration * timestep);
	keyFrameVelocityLimit = std::max({keyFrameVelocityLimit, linear_velocity_deceleration_limit});
	// Find the maximum allowable limits
	float desired_linear_velocity = std::min({curvature_velocity_limit, linear_velocity_acceleration_limit, keyFrameVelocityLimit, max_linear_velocity});
	// Calculates the distance to be traveled in the allotted time frame  
	float deltaS = desired_linear_velocity * timestep;
	// Calculates when T corresponds with the distance that will be traveled
	float t = findTForS(controlPoints, sCurrent, deltaS);
	float turning_velocity_component = curvature(controlPoints, t) * desired_linear_velocity;
	Pose newPoint = findXandY(controlPoints, t);
	poses.push_back(newPoint);
	VelocityLayout newVelo = {desired_linear_velocity, turning_velocity_component, tim};
	velocities.push_back(newVelo);
	prev_t = t;
	curSpeed = desired_linear_velocity;
	return{desired_linear_velocity, turning_velocity_component};
}
float sinc(float x) {
	return std::abs(x) < 1e-5 ? 1.0f : std::sin(x) / x;
}
void moveToPose(std::vector<Pose> inPoses, std::vector<VelocityLayout> inVelocities, float max_linear_velocity, float B, float zeta, float dt) {
	float trackWidth = 0.288925f;
	float omega_max = (2 * max_linear_velocity) / trackWidth;
	float k1 = B;
    bool reverse = true;
	Pose ramPose = inPoses.front();
	ramPose.theta = M_PI + ramPose.theta;
	float ramTim = 0;
	auto leftMotor = FirstOrderWithDeadBand(1/0.860255033623, 0.135466173286, 0.659094734514);
	auto rightMotor = FirstOrderWithDeadBand(1/0.860255033623, 0.135466173286, 0.659094734514);

	float voltsToVelocity = 3.25 * 0.0254 * 600 * M_PI / 60.f;


	// main loop
	for (float i = 0; i < inPoses.size(); i++) {
		ramTim += dt;
		if (reverse) {
            inVelocities[i].linear = -inVelocities[i].linear;
    		inPoses[i].theta += M_PI;
		}
        float k2 = std::sqrt(inVelocities[i].angular * inVelocities[i].angular + B * inVelocities[i].linear * inVelocities[i].linear);
		float error_theta = inPoses[i].theta - ramPose.theta;
		// wrap into [−π,π]:

		error_theta = fmodf(error_theta + (float)M_PI, 2.0f * (float)M_PI) - (float)M_PI;
		
		float dx = inPoses[i].x - ramPose.x;
		float dy = inPoses[i].y - ramPose.y;
		float cosTheta = cosf(ramPose.theta);
		float sinTheta = sinf(ramPose.theta);
		float error_x = sinTheta * dy + cosTheta * dx;
		float error_y = cosTheta * dy - sinTheta * dx;
		float linearOut = inVelocities[i].linear * cos(error_theta) + k1 * error_x;
		
		float angularOut = inVelocities[i].angular + k2 * (inVelocities[i].linear) * sinc(error_theta) * error_y + k1 * error_theta;
 
		float leftVel = linearOut - (angularOut * trackWidth) / 2;
		float rightVel = linearOut + (angularOut * trackWidth) / 2;
	        float leftVolts = leftVel / max_linear_velocity * 12;
	        float rightVolts = rightVel / max_linear_velocity * 12;
		// leftVel = voltsToVelocity * leftMotor.update(leftVolts, dt);
		// rightVel = voltsToVelocity * leftMotor.update(rightVolts, dt);

		float v = (leftVel + rightVel) / 2;
		float w = (rightVel - leftVel) / trackWidth;
		ramPose.x += v * cosf(ramPose.theta) * dt;
		ramPose.y += v * sinf(ramPose.theta) * dt;
		ramPose.theta += w * dt;
		ramsetePoses.push_back(ramPose);
		VelocityLayout newVelo = {v, w, ramTim};
		ramseteVelocities.push_back(newVelo);
	
	}
}
void printVels(std::string splineName, const std::vector<Point>& controlPoints, std::vector<KeyframeVelocitiesXandY> keyFrameVelocityInitList, bool useKeyFrames) {
	std::vector<KeyframeVelocities> keyFrameVelocityList = convertToTFrame(controlPoints, keyFrameVelocityInitList);
	float maxVelocity = 1.94503855166; // meters per second
	float maxAcceleration = 6.67663722193; // meters per second per second
	float initialVelocity = keyFrameVelocityList.front().velocity;
	float exitVelocity = keyFrameVelocityList.back().velocity;
	curSpeed = initialVelocity;
	prev_t = 0;
	tim = 0;
	float decelerationDistance = sFunction(controlPoints, 1) - (pow(exitVelocity, 2) - pow(maxVelocity, 2)) / (-2 * maxAcceleration);

	while (prev_t < 1) {
		twoDimensionalTrapVel(controlPoints, maxVelocity, maxAcceleration, decelerationDistance, initialVelocity, exitVelocity, keyFrameVelocityList, useKeyFrames, 0.01);
	}
	// Output the points in the desired format
	std::cout << "X = [";
	for (size_t i = 0; i < poses.size(); ++i) {
		std::cout << "(" << std::fixed << poses[i].x << "," << poses[i].y << ")";
		if (i < poses.size() - 1) {
			std::cout << ",";
		}
	}
	std::cout << "]" << std::endl;

	std::cout << "L = [";
	for (size_t i = 0; i < velocities.size(); ++i) {
		std::cout << "(" << std::fixed << velocities[i].time << "," << velocities[i].linear << ")";
		if (i < poses.size() - 1) {
			std::cout << ",";
		}
	}
	std::cout << "]" << std::endl;
	std::cout << "A = [";
	for (size_t i = 0; i < velocities.size(); ++i) {
		std::cout << "(" << std::fixed << velocities[i].time << "," << velocities[i].angular << ")";
		if (i < poses.size() - 1) {
			std::cout << ",";
		}
	}
	std::cout << "]" << std::endl;
	moveToPose(poses, velocities, maxVelocity, 2, 0.7, 0.01);
	std::cout << "X_r = [";
	for (size_t i = 0; i < ramsetePoses.size(); ++i) {
		std::cout << "(" << std::fixed << ramsetePoses[i].x << "," << ramsetePoses[i].y << ")";
		if (i < ramsetePoses.size() - 1) {
			std::cout << ",";
		}
	}
	std::cout << "]" << std::endl;

	std::cout << "L_r = [";
	for (size_t i = 0; i < ramseteVelocities.size(); ++i) {
		std::cout << "(" << std::fixed << ramseteVelocities[i].time << "," << ramseteVelocities[i].linear << ")";
		if (i < poses.size() - 1) {
			std::cout << ",";
		}
	}
	std::cout << "]" << std::endl;
	std::cout << "A_r = [";
	for (size_t i = 0; i < ramseteVelocities.size(); ++i) {
		std::cout << "(" << std::fixed << ramseteVelocities[i].time << "," << ramseteVelocities[i].angular << ")";
		if (i < poses.size() - 1) {
			std::cout << ",";
		}
	}
	std::cout << "]" << std::endl;
}
