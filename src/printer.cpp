// printer.cpp
#include "printer.hpp"
#include <iomanip>
#include <ostream>

namespace Printer {

void printPoseVectorDesmos(
	std::ostream& out,
	const std::string& label,
	const std::vector<std::vector<Pose>>& poses
) {
	out << label << "[";
	for (size_t i = 0; i < poses.size(); ++i) {
		for (size_t j = 0; j < poses[i].size(); ++j) {
			out
				<< "("
				<< std::fixed << std::setprecision(6)
				<< poses[i][j].x << ","
				<< poses[i][j].y
				<< ")";
			if (i != poses.size() - 1 || j != poses[i].size() - 1) {
				out << ",";
			}
		}
	}
	out << "]\n";
}

void printVelocityVectorDesmos(
	std::ostream& out,
	const std::string& label,
	const std::vector<std::vector<VelocityLayout>>& vels,
	const std::string& whichField
) {
	out << label << "[";
	for (size_t i = 0; i < vels.size(); ++i) {
		for (size_t j = 0; j < vels[i].size(); ++j) {
			float value = (whichField == "linear")
				? vels[i][j].linear
				: vels[i][j].angular;
			out
				<< "("
				<< std::fixed << std::setprecision(6)
				<< vels[i][j].time << ","
				<< value
				<< ")";
			if (i != vels.size() - 1 || j != vels[i].size() - 1) {
				out << ",";
			}
		}
	}
	out << "]\n";
}

// prints P = {(x, y),(x, y),...}
void printPoseVectorCode(
	std::ostream& out,
	const std::string& label,
	const std::vector<std::vector<Pose>>& poses
) {
	out << label << " {";

	bool first = true;
	for (const auto& row : poses) {
		for (const auto& p : row) {
			if (!first) out << ",";
			out << "("
				<< std::fixed << std::setprecision(6)
				<< p.x << ", " << p.y << ", " << p.theta
				<< ")";
			first = false;
		}
	}

	out << "};\n";
}

// prints V = {(linear, angular),(linear, angular),...}
void printVelocityVectorCode(
	std::ostream& out,
	const std::string& label,
	const std::vector<std::vector<VelocityLayout>>& vels
) {
	out << label << " {";

	bool first = true;
	for (const auto& row : vels) {
		for (const auto& v : row) {
			if (!first) out << ",";
			out << "("
				<< std::fixed << std::setprecision(6)
				<< v.linear << ", " << v.angular
				<< ")";
			first = false;
		}
	}

	out << "};\n";
}
}
