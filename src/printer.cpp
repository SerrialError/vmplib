// printer.cpp
//
// The output boundary. Internal state is carried in double (see types.hpp);
// the values are narrowed to float here, on the way out, because that is the
// precision a Desmos plot or a robot's velocity controller actually consumes.
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
				<< static_cast<float>(poses[i][j].x) << ","
				<< static_cast<float>(poses[i][j].y)
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
			float value = static_cast<float>((whichField == "linear")
				? vels[i][j].linear
				: vels[i][j].angular);
			out
				<< "("
				<< std::fixed << std::setprecision(6)
				<< static_cast<float>(vels[i][j].time) << ","
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
				<< static_cast<float>(p.x) << ", " << static_cast<float>(p.y)
				<< ", " << static_cast<float>(p.theta)
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
				<< static_cast<float>(v.linear) << ", " << static_cast<float>(v.angular)
				<< ")";
			first = false;
		}
	}

	out << "};\n";
}

void printScalarSamplesDesmos(
	std::ostream& out,
	const std::vector<ScalarSample>& samples
) {
	const auto printList = [&](const char* label, double ScalarSample::*field) {
		out << label << "[";
		for (size_t i = 0; i < samples.size(); ++i) {
			if (i != 0) out << ",";
			out << "("
				<< std::fixed << std::setprecision(6)
				<< static_cast<float>(samples[i].time) << ","
				<< static_cast<float>(samples[i].*field)
				<< ")";
		}
		out << "]\n";
	};
	printList("P = ", &ScalarSample::position);
	printList("V = ", &ScalarSample::velocity);
	printList("A = ", &ScalarSample::accel);
}

// prints S = {{position, velocity, accel},{position, velocity, accel},...};
void printScalarSamplesCode(
	std::ostream& out,
	const std::vector<ScalarSample>& samples
) {
	out << "S = {";
	for (size_t i = 0; i < samples.size(); ++i) {
		if (i != 0) out << ",";
		out << "{"
			<< std::fixed << std::setprecision(6)
			<< static_cast<float>(samples[i].position) << ", "
			<< static_cast<float>(samples[i].velocity) << ", "
			<< static_cast<float>(samples[i].accel)
			<< "}";
	}
	out << "};\n";
}
}
