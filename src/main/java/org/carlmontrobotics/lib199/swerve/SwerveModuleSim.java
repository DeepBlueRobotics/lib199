package org.carlmontrobotics.lib199.swerve;

import org.carlmontrobotics.lib199.sim.MockedCANCoder;
import org.carlmontrobotics.lib199.sim.MockedEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Mult;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class SwerveModuleSim {
    private SimDeviceSim driveMotorSim, driveEncoderSim, turnMotorSim, turnEncoderSim;
    private DCMotorSim drivePhysicsSim, turnPhysicsSim;
    private double driveGearing, turnGearing;
    private boolean driveInversion, turnInversion;
    private Timer timer = new Timer();

    /**
     * Constructs a SwerveModuleSim that simulates the physics of a swerve module.
     * 
     * @param drivePortNum the port of the SparkMax drive motor
     * @param driveGearing the gearing reduction between the drive motor and the wheel
     * @param driveInversion whether the drive motor is inverted
     * @param driveMoiKgM2 the effective moment of inertia around the wheel axle (typciall the mass of the robot divided the number of modules times the square of the wheel radius)
     * @param turnMotorPortNum the port of the SparkMax turn motor
     * @param turnEncoderPortNum the port of the CANCoder measuring the module's angle
     * @param turnGearing the gearing reduction between the turn motor and the module
     * @param turnInversion whether the turn motor is inverted
     * @param turnMoiKgM2 the moment of inertia of the part of the module turned by the turn motor (in kg m^2)
     */
    public SwerveModuleSim(int drivePortNum, double driveGearing, boolean driveInversion, double driveMoiKgM2, 
                            int turnMotorPortNum, int turnEncoderPortNum, double turnGearing, boolean turnInversion, double turnMoiKgM2) {
        driveMotorSim = new SimDeviceSim("SparkMax", drivePortNum);
        driveEncoderSim = new SimDeviceSim(driveMotorSim.getName() + "_RelativeEncoder");
        drivePhysicsSim = new DCMotorSim(DCMotor.getNEO(1), driveGearing, driveMoiKgM2);
        this.driveGearing = driveGearing;
        this.driveInversion = driveInversion;

        turnMotorSim = new SimDeviceSim("SparkMax", turnMotorPortNum);
        turnEncoderSim = new SimDeviceSim("CANCoder", turnEncoderPortNum);
        turnPhysicsSim = new DCMotorSim(DCMotor.getNEO(1), turnGearing, turnMoiKgM2);
        this.turnGearing = turnGearing;
        this.turnInversion = turnInversion;
    }

    /**
     * Steps the simulation forward by dtSecs seconds.
     * @param dtSecs seconds to step the simulation forward.
     */
    public void update(double dtSecs) {
        drivePhysicsSim.setInputVoltage(DriverStation.isEnabled() ? driveMotorSim.getDouble("Speed").get()*12.0 : 0.0);
        drivePhysicsSim.update(dtSecs);
        driveEncoderSim.getDouble("Position").set(drivePhysicsSim.getAngularPositionRotations()*MockedEncoder.NEO_BUILTIN_ENCODER_CPR*driveGearing);
        driveEncoderSim.getDouble("Velocity").set(drivePhysicsSim.getAngularVelocityRPM()*MockedEncoder.NEO_BUILTIN_ENCODER_CPR*driveGearing);

        turnPhysicsSim.setInputVoltage(DriverStation.isEnabled() ? turnMotorSim.getDouble("Speed").get()*12.0 : 0.0);
        turnPhysicsSim.update(dtSecs);
        // The -1.0 below is to account for the fact that the CANCoder is mounted such that turning the wheel CCW (as viewed from above) causes
        // the encoder value to decrease. However the turnPhysicsSim's angular position will *increase* under those circumstances.
        // Note that this is independent of turnInversion. turnInversion controls which direction a positive voltage will cause the turn motor
        // to spin. turnInversion should be used to set the motor's inversion so that a positive voltage will spin the wheel CCW (as viewed from above).
        turnEncoderSim.getDouble("count").set(MathUtil.inputModulus(-1.0 * turnPhysicsSim.getAngularPositionRotations(), -0.5, 0.5)*MockedCANCoder.kCANCoderCPR);
    }

    /**
     * Steps the simulation forward by the amount of time that has elapsed since this method was last called.
     */
    public void update() {
        double dtSecs = timer.get();
        timer.restart();
        update(dtSecs);
    }
}
