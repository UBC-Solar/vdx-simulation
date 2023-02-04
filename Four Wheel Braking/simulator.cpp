  /*
    Tire radius: 241.3 mm
    Front Length: 44.86 in
    Rear Length: 37.95 in
    COM Height: 241.3 mm + 2.55 in
    Wheel moment of inertia: 376.86 lbf * in^2

    Hub Moment of Inertia: 0.00736 kg * m^2
    Wheel Moment of Inertia: 0.11028 kg * m^2

    Total Moment of Inertia: ~0.118 kg * m^2

    Mass: 353602.12 grams

    Disk Radius: 3.55
    Drive Disk Radius: Assumed 3.55 for now
*/

#include <algorithm>
#include <iterator>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include "simulator.hpp"
#include "IO.hpp"

int main(void) {
    // Main Simulation Control
    std::string fileLocationFW = "results/FW_00.csv";
    std::string fileLocationBW = "results/BW.csv";

    const double timeStep = 0.0001;
    const double simulationTime = 30;
    const uint64_t iterations = (uint64_t)(simulationTime / timeStep) + 1;

    const bool debugMode = false;
    const bool writeMode = true;

    pacejkaCoefficients PC(10, 1.9, 1, 0.97); // B, C, D, E as determined from experimental tyre data
    std::function<double(const double&)> pedalInput = [](const double& time) -> double { return 5000 * std::log(time + 1); };

    // Car Parameters derived from Brightside Rev. 1 model using centre of mass function and measurements to the rotational line of each axle.
    const double Mass = 354;
    const double Front_Length = 1.139;
    const double Rear_Length = 0.964;
    const double COM_Height = 0.3061;

    const double Tire_Radius = 0.2413;
    const double Disk_Radius = 0.0902;
    const double Wheel_MomentOfInertia = 0.118;
    const double DiskBrake_COF = 0.4;
    const double DiskBrake_BoreDiameter = 0.0254;
    const double MasterCylinder_BoreDiameter = 0.022225;

    const double ProportioningValve_Reduction = 0.57;

    // Driving Parameters
    const double InitialVelocity = 28;
    const double InitialAcceleration = 0;
    const double InitialDistance = 0;
    const double InitialWheelSlip = 0;

    const double InitialAngularVelocity = InitialVelocity / Tire_Radius;
    const double InitialBrakingForce = 0;

    // Create car, mastercylinder, and proportioning valves for simulation
    Car BrightSide(Mass, Front_Length, Rear_Length, COM_Height,InitialVelocity, InitialAcceleration, timeStep);
    MasterCylinder cylinderOne(MasterCylinder_BoreDiameter);
    ProportioningValve propOne(ProportioningValve_Reduction, cylinderOne.returnPressure());
    Wheel wheel1(Tire_Radius, Disk_Radius, Wheel_MomentOfInertia, PC, DiskBrake_COF, InitialWheelSlip, InitialAngularVelocity, BrightSide.returnFrontForce(), InitialBrakingForce, DiskBrake_BoreDiameter, propOne.returnPressure(), timeStep);
    Wheel wheel2(Tire_Radius, Disk_Radius, Wheel_MomentOfInertia, PC, DiskBrake_COF, InitialWheelSlip, InitialAngularVelocity, BrightSide.returnRearForce(), InitialBrakingForce, DiskBrake_BoreDiameter, propOne.returnPressure(), timeStep);

    // Attach prop valves, master cylinders, and wheels to car
    BrightSide.setInputForce() = pedalInput;

    BrightSide.addMasterCylinder(&cylinderOne);
    BrightSide.addPropValve(&propOne);
    BrightSide.addWheel(&wheel1);
    BrightSide.addWheel(&wheel2);

    // Iterate for the given number of requested steps
    uint64_t it = 0;
    for (; it < iterations; it++) {
        BrightSide.step();
        it = BrightSide.updateIteration();
        if (*BrightSide.returnVelocity() <= 0.5) break;
    }

    // Write wheel data to a file
    std::vector<std::vector<double>> outputStream;
    auto velRecord = BrightSide.returnVelocityRecord();

    if (writeMode) {
    outputStream.push_back(velRecord);
    wheel1.returnWheelData(outputStream);
    bool successfulWrite = output(outputStream, fileLocationFW);
    outputStream.clear();

    outputStream.push_back(velRecord);
    wheel2.returnWheelData(outputStream);
    successfulWrite = output(outputStream, fileLocationBW);
    }

    // Print basic data to console
    std::cout << std::endl;
    std::cout << "----------------------------------" << std::endl << std::endl;
    std::cout << "Final Velocity:    " << *BrightSide.returnVelocity() << " m/s" << std::endl;
    std::cout << "Iterations:        " << it - 1 << std::endl;
    std::cout << "Time:              " << timeStep * (it - 1) << " s" << std::endl;
    std::cout << "Acceleration:      " << (InitialVelocity - *BrightSide.returnVelocity()) / (it*timeStep) << " m/s" << std::endl;
    std::cout << "Acceleration:      " << (InitialVelocity - *BrightSide.returnVelocity()) / (it*timeStep*S_GR) << " G's" << std::endl;
    if (debugMode) std::cout << "Minimum Velocity:  " << *std::min_element(velRecord.begin(), velRecord.end()) << " m/s" << std::endl;
    std::cout << std::endl;
    std::cout << "Slip Ratio" << std::endl;
    std::cout << "Front:            " << Wheel::maximumSlip(wheel1) << std::endl;
    std::cout << "Rear:             " << Wheel::maximumSlip(wheel2) << std::endl;
    if (debugMode) {
    std::cout << std::endl;
    if (writeMode) std::cout << outputStream[0].size() << " lines written to output." << std::endl;
    }
    std::cout << std::endl;
    std::cout << "----------------------------------" << std::endl;

    return 0;
}