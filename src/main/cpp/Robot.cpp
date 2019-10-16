#include <iostream>
#include "Robot.h"

#define INPUT_DEAD_BAND 0.08

#define WHEEL_NON_LINEARITY 0.6
#define NONLINEAR_PASSES 3
#define NEGATIVE_INERTIA_TURN_SCALAR 2.5

#define PI 3.14159265358979323846
#define HALF_PI (PI / 2.0)

#define NEGATIVE_INERTIA_THRESHOLD 0.65

#define NEGATIVE_INERTIA_FAR_SCALAR 5.0
#define NEGATIVE_INERTIA_CLOSE_SCALAR 3.0

#define QUICK_STOP_DEAD_BAND 0.5
#define QUICK_STOP_WEIGHT 0.45
#define QUICK_STOP_SCALAR 5.0
#define LOW_SENSITIVITY 0.625
#define WHEEL_QUICK_TURN_SCALAR 0.65

void Robot::RobotInit() {
    m_RightSlaveOne.Follow(m_RightMaster);
    m_RightSlaveTwo.Follow(m_RightMaster);
    m_LeftSlaveOne.Follow(m_LeftMaster);
    m_LeftSlaveTwo.Follow(m_LeftMaster);
    m_SliderMaster.ConfigClosedloopRamp(0.2, 500);
}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {

    double
            throttle = handleDeadBand(-m_DriveStick.GetY()),
            wheel = handleDeadBand(m_TurnStick.GetX()),
            slide;
    if (m_TurnStick.GetPOV() == 90) {
        slide = 0.4;
    } else if (m_TurnStick.GetPOV() == 270) {
        slide = -0.4;
    } else {
        slide = 0.0;
    }

    // P-Drive
//    m_RightMaster.Set(ctr::ControlMode::PercentOutput, throttle + wheel);
//    m_LeftMaster.Set(ctr::ControlMode::PercentOutput, throttle - wheel);
    m_SliderMaster.Set(ctr::ControlMode::PercentOutput, slide);

    // Constant-curvature drive
//    bool isQuickTurn = m_DriveStick.GetTrigger();
    bool isQuickTurn = m_TurnStick.GetTrigger();
    double negativeInertia = wheel - m_LastWheel;
    m_LastWheel = wheel;

    double denominator = std::sin(HALF_PI * WHEEL_NON_LINEARITY);
    for (int i = 0; i < NONLINEAR_PASSES; i++) {
        wheel = std::sin(HALF_PI * WHEEL_NON_LINEARITY * wheel) / denominator;
    }

    double negativeInertiaScalar;
    if (wheel * negativeInertia > 0) {
        negativeInertiaScalar = NEGATIVE_INERTIA_TURN_SCALAR;
    } else {
        if (std::fabs(wheel) > NEGATIVE_INERTIA_THRESHOLD) {
            negativeInertiaScalar = NEGATIVE_INERTIA_FAR_SCALAR;
        } else {
            negativeInertiaScalar = NEGATIVE_INERTIA_CLOSE_SCALAR;
        }
    }

    double negativeInertiaPower = negativeInertia * negativeInertiaScalar;
    m_NegativeInertiaAccumulator += negativeInertiaPower;

    wheel += m_NegativeInertiaAccumulator;
    if (m_NegativeInertiaAccumulator > 1) {
        m_NegativeInertiaAccumulator -= 1;
    } else if (m_NegativeInertiaAccumulator < -1) {
        m_NegativeInertiaAccumulator += 1;
    } else {
        m_NegativeInertiaAccumulator = 0;
    }

    double linearPower = throttle;
    double overPower, angularPower;
    if (isQuickTurn) {
        if (std::fabs(linearPower) < QUICK_STOP_DEAD_BAND) {
            double alpha = QUICK_STOP_WEIGHT;
            m_QuickStopAccumulator = (1 - alpha) * m_QuickStopAccumulator + alpha * clamp01(wheel) * QUICK_STOP_SCALAR;
        }
        overPower = 1.0;
        angularPower = wheel * WHEEL_QUICK_TURN_SCALAR;
    } else {
        overPower = 0.0;
        angularPower = std::fabs(throttle) * wheel * LOW_SENSITIVITY - m_QuickStopAccumulator;
        if (m_QuickStopAccumulator > 1) {
            m_QuickStopAccumulator -= 1;
        } else if (m_QuickStopAccumulator < -1) {
            m_QuickStopAccumulator += 1;
        } else {
            m_QuickStopAccumulator = 0.0;
        }
    }

    double rightOutput = linearPower, leftOutput = linearPower;
    leftOutput += angularPower;
    rightOutput -= angularPower;

    if (leftOutput > 1.0) {
        rightOutput -= overPower * (leftOutput - 1.0);
        leftOutput = 1.0;
    } else if (rightOutput > 1.0) {
        leftOutput -= overPower * (rightOutput - 1.0);
        rightOutput = 1.0;
    } else if (leftOutput < -1.0) {
        rightOutput += overPower * (-1.0 - leftOutput);
        leftOutput = -1.0;
    } else if (rightOutput < -1.0) {
        leftOutput += overPower * (-1.0 - rightOutput);
        rightOutput = -1.0;
    }

//    std::cout << "Left " << leftOutput << std::endl;
//    std::cout << "Right " << rightOutput << std::endl;
//    std::cout << "Linear " << linearPower << std::endl;
//    std::cout << "Throttle " << throttle << std::endl;
//    std::cout << "Angular " << angularPower << std::endl;

    m_RightMaster.Set(ctr::ControlMode::PercentOutput, -rightOutput);
    m_LeftMaster.Set(ctr::ControlMode::PercentOutput, leftOutput);
}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

double Robot::handleDeadBand(double input) {
    return std::fabs(input) > INPUT_DEAD_BAND ? input : 0.0;
}

double Robot::clamp01(double value) {
    return std::min(1.0, std::max(-1.0, value));
}

void Robot::DisabledInit() {
    m_LastWheel = 0.0;
    m_NegativeInertiaAccumulator = 0.0;
    m_QuickStopAccumulator = 0.0;
}

#ifndef RUNNING_FRC_TESTS

int main() { return frc::StartRobot<Robot>(); }

#endif
