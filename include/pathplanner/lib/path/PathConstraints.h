#pragma once

#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/voltage.h>
#include <wpi/json.h>
#include <limits>

namespace pathplanner {
class PathConstraints {
public:
	/**
	 * Create a new path constraints object
	 *
	 * @param maxVel Max linear velocity (M/S)
	 * @param maxAccel Max linear acceleration (M/S^2)
	 * @param maxAngularVel Max angular velocity (Deg/S)
	 * @param maxAngularAccel Max angular acceleration (Deg/S^2)
	 * @param nominalVoltage The nominal battery voltage (Volts)
	 * @param unlimited Should the constraints be unlimited
	 */
	constexpr PathConstraints(units::meters_per_second_t maxVel,
			units::meters_per_second_squared_t maxAccel,
			units::radians_per_second_t maxAngularVel,
			units::radians_per_second_squared_t maxAngularAccel,
			units::volt_t nominalVoltage = 12_V, bool unlimited = false,
		    bool deceleration_valid = false, units::meters_per_second_squared_t maxDeceleration = 0_mps_sq) : m_maxVelocity(
			maxVel), m_maxAcceleration(maxAccel), m_maxAngularVelocity(
			maxAngularVel), m_maxAngularAcceleration(maxAngularAccel), m_nominalVoltage(
			nominalVoltage), m_unlimited(unlimited), m_deceleration_valid(deceleration_valid),
			m_maxDeceleration(maxDeceleration) {
	}

	/**
	 * Create a path constraints object from json
	 *
	 * @param json json reference representing a path constraints object
	 * @return The path constraints defined by the given json
	 */
	static PathConstraints fromJson(const wpi::json &json);

	/**
	 * Get unlimited PathConstraints
	 *
	 * @param nominalVoltage The nominal battery voltage (Volts)
	 * @return Unlimited constraints
	 */
	static constexpr PathConstraints unlimitedConstraints(
			units::volt_t nominalVoltage) {
		double inf = std::numeric_limits<double>::infinity();
		return PathConstraints(units::meters_per_second_t { inf },
				units::meters_per_second_squared_t { inf },
				units::radians_per_second_t { inf },
				units::radians_per_second_squared_t { inf }, nominalVoltage,
				true);
	}

	/**
	 * Get the max linear velocity
	 *
	 * @return Max linear velocity (M/S)
	 */
	constexpr units::meters_per_second_t getMaxVelocity() const {
		return m_maxVelocity;
	}

	/**
	 * Get the max linear acceleration
	 *
	 * @return Max linear acceleration (M/S^2)
	 */
	constexpr units::meters_per_second_squared_t getMaxAcceleration() const {
		return m_maxAcceleration;
	}

	/**
	 * Get the max linear deceleration
	 *
	 * @return Max linear deceleration (M/S^2)
	 */
	constexpr units::meters_per_second_squared_t getMaxDeceleration() const {
		if (m_deceleration_valid)
			return m_maxDeceleration;
		return m_maxAcceleration;
	}

	/**
	 * Get the max angular velocity
	 *
	 * @return Max angular velocity (Rad/S)
	 */
	constexpr units::radians_per_second_t getMaxAngularVelocity() const {
		return m_maxAngularVelocity;
	}

	/**
	 * Get the max angular acceleration
	 *
	 * @return Max angular acceleration (Rad/S^2)
	 */
	constexpr units::radians_per_second_squared_t getMaxAngularAcceleration() const {
		return m_maxAngularAcceleration;
	}

	/**
	 * Get the nominal voltage
	 *
	 * @return Nominal Voltage (Volts)
	 */
	constexpr units::volt_t getNominalVoltage() const {
		return m_nominalVoltage;
	}

	constexpr bool isUnlimited() const {
		return m_unlimited;
	}

	bool operator==(const PathConstraints &other) const {
		return std::abs(m_maxVelocity() - other.m_maxVelocity()) < 1E-9
				&& std::abs(m_maxAcceleration() - other.m_maxAcceleration())
						< 1E-9
				&& std::abs(
						m_maxAngularVelocity() - other.m_maxAngularVelocity())
						< 1E-9
				&& std::abs(
						m_maxAngularAcceleration()
								- other.m_maxAngularAcceleration()) < 1E-9
				&& std::abs(m_nominalVoltage() - other.m_nominalVoltage())
						< 1E-9 && m_unlimited == other.m_unlimited
				&& (!m_deceleration_valid || std::abs(m_maxDeceleration() - other.m_maxDeceleration())
				        < 1E-9);
	}

private:
	units::meters_per_second_t m_maxVelocity;
	units::meters_per_second_squared_t m_maxAcceleration;
	units::meters_per_second_squared_t m_maxDeceleration = 0_mps_sq;
	bool m_deceleration_valid = false;
	units::radians_per_second_t m_maxAngularVelocity;
	units::radians_per_second_squared_t m_maxAngularAcceleration;
	units::volt_t m_nominalVoltage;
	bool m_unlimited;
};
}
