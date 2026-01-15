package org.carlmontrobotics.lib199.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class SwerveModuleSim {
    private SimDeviceSim driveMotorSim, driveEncoderSim, turnMotorSim, turnEncoderSim;
    private DCMotorSim drivePhysicsSim, turnPhysicsSim;
    private double driveGearing;
    private Timer timer = new Timer();

    /**
     * Constructs a SwerveModuleSim that simulates the physics of a swerve module.
     *
     * @param drivePortNum the port of the SparkMax drive motor
     * @param driveGearing the gearing reduction between the drive motor and the wheel
     * @param driveMoiKgM2 the effective moment of inertia around the wheel axle (typciall the mass of the robot divided the number of modules times the square of the wheel radius)
     * @param turnMotorPortNum the port of the SparkMax turn motor
     * @param turnEncoderPortNum the port of the CANCoder measuring the module's angle
     * @param turnGearing the gearing reduction between the turn motor and the module
     * @param turnMoiKgM2 the moment of inertia of the part of the module turned by the turn motor (in kg m^2)
     */
    public SwerveModuleSim(int drivePortNum, double driveGearing, double driveMoiKgM2,
                            int turnMotorPortNum, int turnEncoderPortNum, double turnGearing, double turnMoiKgM2) {
        driveMotorSim = new SimDeviceSim("CANMotor:CANSparkMax", drivePortNum);
        driveEncoderSim = new SimDeviceSim("CANEncoder:CANSparkMax", drivePortNum);
        DCMotor dcmotor = DCMotor.getNEO(1);
        drivePhysicsSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(dcmotor, driveMoiKgM2, driveGearing),  dcmotor);//FIXME WHAT DO WE WANT THE MEASUREMENT STDDEVS TO BE? (LAST ARG)
        this.driveGearing = driveGearing;

        turnMotorSim = new SimDeviceSim("CANMotor:CANSparkMax", turnMotorPortNum);
        turnEncoderSim = new SimDeviceSim("CANDutyCycle:CANCoder", turnEncoderPortNum);
        turnPhysicsSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(dcmotor, turnMoiKgM2, turnGearing),
            dcmotor);
    }

    /**
     * Steps the simulation forward by dtSecs seconds.
     * @param dtSecs seconds to step the simulation forward.
     */
    public void update(double dtSecs) {
        drivePhysicsSim.setInputVoltage(DriverStation.isEnabled() ? driveMotorSim.getDouble("percentOutput").get()*12.0 : 0.0);
        drivePhysicsSim.update(dtSecs);
        driveEncoderSim.getDouble("position").set(drivePhysicsSim.getAngularPositionRotations()*driveGearing);
        driveEncoderSim.getDouble("velocity").set(drivePhysicsSim.getAngularVelocityRPM()/60.0*driveGearing);

        turnPhysicsSim.setInputVoltage(DriverStation.isEnabled() ? turnMotorSim.getDouble("percentOutput").get()*12.0 : 0.0);
        turnPhysicsSim.update(dtSecs);
        // The -1.0 below is to account for the fact that the CANCoder is mounted such that turning the wheel CCW (as viewed from above) causes
        // the encoder value to decrease. However the turnPhysicsSim's angular position will *increase* under those circumstances.
        turnEncoderSim.getDouble("position").set(MathUtil.inputModulus(-1.0 * turnPhysicsSim.getAngularPositionRotations(), -0.5, 0.5));
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
