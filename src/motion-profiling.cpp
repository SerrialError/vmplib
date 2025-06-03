#include "motion-profiling.hpp"
#include "types.hpp"
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
