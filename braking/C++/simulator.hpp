#ifndef SIMULATOR_HPP

#include <iostream>
#include <vector>
#include <functional>
#include <iterator>
#include <cmath>

const double S_PI = 3.14159265358979323846;
const double S_GR = 9.81;

struct pacejkaCoefficients {
    pacejkaCoefficients(const double iniB, const double iniC, const double iniD, const double iniE) : B(iniB), C(iniC), D(iniD), E(iniE) {}
    pacejkaCoefficients() = default;
    const double B, C, D, E;  
};

class Wheel {
public:
    Wheel(const double& iniRadius, const double& iniDiskRadius, const double& iniMomentOfInertia, const pacejkaCoefficients& iniPC, const double& iniDiskCoF, 
    const double& iniSlip, const double& iniAVel, double* iniVertForce, const double& iniHorForce, const double& iniDiameter, double* iniPressure, const double& iniTimeStep) {
        radius = iniRadius;
        diskRadius = iniDiskRadius;
        momentOfInertia = iniMomentOfInertia;

        PC_B = iniPC.B;
        PC_C = iniPC.C;
        PC_D = iniPC.D;
        PC_E = iniPC.E;

        diskCoF = iniDiskCoF;
        slip = iniSlip;
        angularVelocity = iniAVel;

        forceVertical = iniVertForce;
        forceHorizontal = iniHorForce;

        pistonDiameter = iniDiameter;
        boreArea = (0.25) * S_PI * (pistonDiameter * pistonDiameter);

        inputPressure = iniPressure;
        timeStep = iniTimeStep;
    }

    inline void updatePCoF() {
        PCoFRecord.push_back(PCoF);
        PCoF = PC_D * std::sin( PC_C * std::atan( PC_B * slip - (PC_E * ( PC_B * slip - std::atan( PC_B * slip )))));
    }

    inline void updateSlip() { 
        slipRecord.push_back(slip);
        if (velocity != 0) slip = (radius * angularVelocity) / std::abs(velocity) - 1;
    }

    inline void updateAngularVelocity() {
        angularAccelerationRecord.push_back(angularAcceleration);
        angularAcceleration = ( 1 / momentOfInertia ) * (std::abs(forceHorizontal * radius) - brakeMoment);

        angularVelocityRecord.push_back(angularVelocity);
        angularVelocity += angularAcceleration * timeStep;
        angularVelocity = (angularVelocity < 0) ? 0 : angularVelocity;
    }

    inline void updateBraking() {
        brakeForceRecord.push_back(brakeForce);
        brakeForce = boreArea * (*inputPressure);

        brakeMomentRecord.push_back(brakeMoment);
        brakeMoment = brakeForce * diskRadius * diskCoF;
    }

    inline double returnForceHorizontal() { return forceHorizontal; }
    inline double* returnForceVertical() { return forceVertical; }

    inline double& defineInputPressure() { return *inputPressure; }
    inline double& defineVelocity() { return velocity; }
    inline double& defineTimeStep() { return timeStep; }

    inline std::vector<double> returnPCoFRecord() { return PCoFRecord; }
    inline std::vector<double> returnSlipRecord() { return slipRecord; }
    inline std::vector<double> returnAngularVelocityRecord() { return angularVelocityRecord; }
    inline std::vector<double> returnAngularAccelerationRecord() { return angularAccelerationRecord; }
    inline std::vector<double> returnForceVerticalRecord() { return forceVerticalRecord; }
    inline std::vector<double> returnForceHorizontalRecord() { return forceHorizontalRecord; }
    inline std::vector<double> returnBrakeForceRecord() { return brakeForceRecord; }
    inline std::vector<double> returnBrakeMomentRecord() { return brakeMomentRecord; }

    inline void returnWheelData(std::vector<std::vector<double>>& vec) {
        // velocity
        vec.push_back(angularVelocityRecord);
        vec.push_back(slipRecord);
        vec.push_back(forceHorizontalRecord);
        vec.push_back(forceVerticalRecord);
        vec.push_back(brakeForceRecord);
        vec.push_back(angularAccelerationRecord);

    }

    static inline double maximumSlip(Wheel& ptr) {
        double tempSlip = *std::min_element(ptr.slipRecord.begin(), ptr.slipRecord.end());
        if (tempSlip < 0) return tempSlip;
        return *std::max_element(ptr.slipRecord.begin(), ptr.slipRecord.end());
    }

    double step();

private:
    double radius;
    double diskRadius;
    double momentOfInertia;

    double PC_B;
    double PC_C;
    double PC_D;
    double PC_E;

    double PCoF;

    double diskCoF;

    double slip;
    double angularVelocity;
    double angularAcceleration;

    double *forceVertical = nullptr;
    double forceHorizontal = 0;

    double pistonDiameter;
    double boreArea;
    double brakeForce = 0;
    double brakeMoment = 0;

    std::vector<double> PCoFRecord;
    std::vector<double> slipRecord;
    std::vector<double> angularVelocityRecord;
    std::vector<double> angularAccelerationRecord;
    std::vector<double> forceVerticalRecord;
    std::vector<double> forceHorizontalRecord;
    std::vector<double> brakeForceRecord;
    std::vector<double> brakeMomentRecord;

    double velocity;

    double* inputPressure = nullptr;
    double timeStep;

    char frontWheel;
};

double Wheel::step() { // Make a single step of the wheel
    updatePCoF();
    updateBraking();

    forceHorizontalRecord.push_back(forceHorizontal);
    forceHorizontal = (*forceVertical) * PCoF;

    forceVerticalRecord.push_back(*forceVertical);
   
    updateAngularVelocity();
    updateSlip();

    return forceHorizontal;
}

class MasterCylinder {
public:
    MasterCylinder() = default;
    MasterCylinder(const double& iniBoreDiameter) {
        boreDiameter = iniBoreDiameter;
        boreArea *= 0.25 * S_PI * std::pow(boreDiameter, 2);
    }

    inline void updateForce() {
        pressureRecord.push_back(pressure);
        pressure = (pedalInputForce) / boreArea;
    }

    inline void setPedalInputForce(double& input) { pedalInputForce = input; }
    inline double* returnPressure() { return &pressure; }

private:
    double boreDiameter;
    double boreArea = 1;
    double pressure = 0;

    std::vector<double> pressureRecord;
    double pedalInputForce;
};

class ProportioningValve {
public:
    ProportioningValve() = default;
    ProportioningValve(const double& iniReduction, double *iniPressureRef) : reductionPercentage(iniReduction), inputPressure(iniPressureRef) {}
    ProportioningValve(const double& iniReductionOne, const double& iniReductionTwo, double* iniPressureRef) {
        reductionPercentage = iniReductionOne * iniReductionTwo;
        inputPressure = iniPressureRef;
    }

    inline void updatePressure() { outputPressure = (*inputPressure) * reductionPercentage; }
    inline double* returnPressure() { return &outputPressure; }

private:
    double reductionPercentage;
    double *inputPressure = nullptr;
    double outputPressure = 0;
};

class Car {
public:
    Car(const double& iniMass, const double& iniFLength, const double& iniRLength, const double& iniHeight, const double& iniVel, const double& iniAcc, const double& iniTimeStep) {
        mass = iniMass;
        velocity = iniVel;
        acceleration = iniAcc;
        centreOfMassHeight = iniHeight;
        frontLength = iniFLength;
        rearLength = iniRLength;
        
        timeStep = iniTimeStep;
    }

    inline void addMasterCylinder(MasterCylinder* cylinder) { masterCylinders.push_back(cylinder); }
    inline void addPropValve(ProportioningValve* propValve) { proportioningValves.push_back(propValve); }
    inline void addWheel(Wheel* wheel) { wheels.push_back(wheel); }

    void step();

    inline double* returnVelocity() { return &velocity; }
    inline double* returnFrontForce() { return &frontWheelForce; }
    inline double* returnRearForce() { return &rearWheelForce; }
    inline double updateIteration() { return ++currentIteration; }
    inline std::vector<double> returnVelocityRecord() { return velocityRecord; }
    inline std::function<double(const double&)>& setInputForce() { return driverInput; }

private:
    double mass;

    double velocity;
    double acceleration;
    double distance = 0;

    double centreOfMassHeight;
    double frontLength;
    double rearLength;

    std::vector<double> velocityRecord;
    std::vector<double> accelerationRecord;
    std::vector<double> distanceRecord;

    double frontWheelForce;
    double rearWheelForce;

    double decelerationForce = 0;
    double front_backBalance;

    double pedalInputForce;

    std::vector<Wheel*> wheels;
    std::vector<MasterCylinder*> masterCylinders;
    std::vector<ProportioningValve*> proportioningValves;

    double timeStep;
    double currentIteration;

    std::function<double(const double&)> driverInput;
};

void Car::step() {
    // Calculate weight transfer
    rearWheelForce = (1/(rearLength + frontLength)) * (frontLength * mass * S_GR + centreOfMassHeight*(decelerationForce));
    frontWheelForce = mass * S_GR - rearWheelForce;

    decelerationForce = 0;

    // Calculate master cylinder input force
    pedalInputForce = driverInput((timeStep) * currentIteration);
    for (auto& cylinder : masterCylinders) {
        cylinder->setPedalInputForce(pedalInputForce);
        cylinder->updateForce();
    }

    // Calculate proportioning valve input and output
    for (auto& propValve : proportioningValves) {
        propValve->updatePressure();
    }

    // Calculate slip of each wheel
    for (auto& wheel : wheels) {
        wheel->defineVelocity() = this->velocity;
        decelerationForce += wheel->step();
    }

    // Calculate new car acceleration with force from each car.
    accelerationRecord.push_back(acceleration);
    acceleration = (decelerationForce) / mass;

    velocityRecord.push_back(velocity);
    velocity += acceleration * (timeStep);
}

#endif // SIMULATOR_HPP