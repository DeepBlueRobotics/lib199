/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.carlmontrobotics.lib199;

import java.util.function.Consumer;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight {

    public static final ObjectMapper JSON_MAPPER = new ObjectMapper();

    public enum Mode {
        DIST, STEER, TARGET
    }

    public final Config config = new Config();
    /**
     * Whether the limelight has any valid targets (0 or 1)
     */
    private double tv;
    /**
     * Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
     */
    private double txDeg;
    /**
     * Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
     */
    private double tyDeg;
    /**
     * Target Area (0% of image to 100% of image)
     */
    private double ta;
    /**
     * The previous value of txDeg
     */
    private double prev_txDeg = 1.0;
    // Mounting angle is the angle of the limelight (angled up = +, angled down = -)
    private double mountingAngleDeg;

    private TurnDirection idleTurnDirection = TurnDirection.CW;

    private PIDController pidController;
    private boolean newPIDLoop = false;
    public Limelight(){
        this("limelight", 0.02);
    }
    public Limelight(String ntName) {
        this(ntName, 0.02);
    }
    public Limelight(double period) {
        this("limelight", period);
    }

    public Limelight(String ntName, double period) {
        config.ntName = ntName;
        double[] pidValues = config.pidSteeringValues;
        pidController = new PIDController(pidValues[0], pidValues[1], pidValues[2], period);
        pidController.setSetpoint(0);
        pidController.setTolerance(config.steeringToleranceDegs);
    }

    /*
     * Determine the mounting angle of the camera given a vision target and its
     * known distance, height off of the ground, and the height of the camera off of
     * the ground.
     */
    public double determineMountingAngle(double distance, double cameraHeight, double objectHeight) {
        // NOTE: ty may be negative.
        tyDeg = NetworkTableInstance.getDefault().getTable(config.ntName).getEntry("ty").getDouble(0.0);
        mountingAngleDeg = Math.atan((cameraHeight - objectHeight) / distance) * 180 / Math.PI - tyDeg;
        return mountingAngleDeg;
    }

    /*
     * Determine the distance an object in the robot's reference frame given the
     * camera's height off of the ground and the object's height off of the ground.
     * Output is {forward distance (x), strafe distance (y)}. cameraHeight is the
     * height of the base of the camera from ground level. objectHeight is the
     * height of the base of the object from ground level.
     */
    public double[] determineObjectDist(double cameraHeight, double objectHeight, double cameraAngle) {
        // angle b/t where limelight is looking, and target in x direction
        txDeg = NetworkTableInstance.getDefault().getTable(config.ntName).getEntry("tx").getDouble(0.0);
        // angle b/t where limelight is looking, and target in y direction
        tyDeg = NetworkTableInstance.getDefault().getTable(config.ntName).getEntry("ty").getDouble(0.0);
        // angle b/t limelight's looking line and flat
        mountingAngleDeg = cameraAngle;
        // diff b/t height of camera and height of target
        double diff = cameraHeight - objectHeight;
        // distance (parallel to ground) b/t limelight and target (does not include
        // height difference)
        double forward = Math.abs(diff / (Math.tan((mountingAngleDeg + tyDeg) / 180 * Math.PI)));
        putValue("forward", forward);
        // hypotenuse of height difference and depth difference (ignores left & right
        // difference) between limelight and target
        double hypotenuse = Math.sqrt(forward * forward + diff * diff);
        // left and right distance b/t target and limelight (only x difference, does not
        // inlcude height or depth)
        double strafe = Math.tan(txDeg / 180 * Math.PI) * hypotenuse;
        putValue("strafe", strafe);
        return new double[] { forward, strafe };
    }

    // Adjusts the distance between a vision target and the robot. Uses basic PID
    // with the ty value from the network table.
    public double distanceAssist() {
        tv = NetworkTableInstance.getDefault().getTable(config.ntName).getEntry("tv").getDouble(0.0);
        ta = NetworkTableInstance.getDefault().getTable(config.ntName).getEntry("ta").getDouble(0.0);
        putValue("tyDeg", tyDeg);
        double adjustment = 0.0;
        double area_threshold = config.areaThresholdPercentage;
        double Kp = config.kP;

        if (tv == 1.0) {
            adjustment = (area_threshold - ta) * Kp;
        }
        adjustment = Math.signum(adjustment) * Math.min(Math.abs(adjustment), config.maxSteeringAdjustment);
        return adjustment;
    }

    // Adjusts the angle facing a vision target. Uses basic PID with the tx value
    // from the network table.
    public double steeringAssist() {
        tv = NetworkTableInstance.getDefault().getTable(config.ntName).getEntry("tv").getDouble(0.0);
        txDeg = NetworkTableInstance.getDefault().getTable(config.ntName).getEntry("tx").getDouble(0.0);
        ta = NetworkTableInstance.getDefault().getTable(config.ntName).getEntry("ta").getDouble(0.0);
        putValue("txDeg", txDeg);
        putValue("tv", tv);

        txDeg = Double.isNaN(txDeg) ? 0 : txDeg;
        double[] pidValues = config.pidSteeringValues;
        pidController.setPID(pidValues[0], pidValues[1], pidValues[2]);
        pidController.setTolerance(config.steeringToleranceDegs);
        double adjustment = 0.0;

        if (tv == 1.0) {
            adjustment = pidController.calculate(txDeg);
            prev_txDeg = txDeg;

            if (!newPIDLoop) {
                newPIDLoop = true;
                pidController.setSetpoint(Math.signum(prev_txDeg) * config.backlashSteeringOffsetDegs);
            }

            adjustment = Math.copySign(Math.min(Math.abs(adjustment), config.maxSteeringAdjustment), txDeg);
        } else {
            newPIDLoop = false;
            pidController.reset();
            adjustment = Math.copySign(config.steeringFactor, idleTurnDirection.sign);
        }
        putValue("adjustment", adjustment);
        return adjustment;
    }

    public boolean isAligned() {
        return pidController.atSetpoint();
    }

    // Combination of distance assist and steering assist
    public double[] autoTarget() {
        double dist_assist = distanceAssist();
        double steer_assist = steeringAssist();
        double[] params = { dist_assist + steer_assist, dist_assist - steer_assist };
        return params;
    }

    public PIDController getPIDController() {
        return pidController;
    }
    public void putValue(String valueName, double value){
      SmartDashboard.putNumber("Limelight("+ config.ntName + "), (" + valueName + ")", value);
    }

    public TurnDirection getIdleTurnDirection() {
        return idleTurnDirection;
    }

    public void setIdleTurnDirection(TurnDirection direction) {
        idleTurnDirection = direction;
    }

    /**
     * Get the JSON dump from the limelight. This method returns via a callback because of the high latency observed in JSON parsing.
     * Keep in mind that the callback will be called asynchronously.
     * 
     * @param onSuccess The callback to run if the JSON dump is successful
     * @param onFailure  The callback to run if an error occurs
     */
    public void getJsonDump(Consumer<LimelightJsonDump> onSuccess, Consumer<Exception> onFailure) {
        new Thread(() -> {
            try {
                onSuccess.accept(JSON_MAPPER.readValue(JSON_MAPPER.readTree(NetworkTableInstance.getDefault().getTable(config.ntName).getEntry("json").getString(null)).elements().next().toString(), LimelightJsonDump.class));
            } catch (Exception e) {
                onFailure.accept(e);
            }
        }).start();
    }

    public Pose3d getTransform(Transform transform) {
        double[] rawData = NetworkTableInstance.getDefault().getTable(config.ntName).getEntry(transform.name().toLowerCase()).getDoubleArray(new double[6]);
        return new Pose3d(rawData[0], rawData[1], rawData[2], new Rotation3d(Math.toRadians(rawData[3]), Math.toRadians(rawData[4]), Math.toRadians(rawData[5])));
    }

    public static class Config {
      /**
       * Limelight name (if using more than one Limelight)
       */ 
      public String ntName = "limelight";
      /**
       *  PID values for Limelight steering
       */ 
      public double[] pidSteeringValues = { 0.01, 0.03, 0 }; 
      /**
       * Tolerance for PID controller for steering (Degrees)
       */
      public double steeringToleranceDegs = 0.01;
      /**
       * Steering adjustment when it does not see the target
       */
      public double steeringFactor = 0.25;
      /**
       * Max value for adjustment
       */
      public double maxSteeringAdjustment = 1.0;
      /**
       * Setpoint of txDeg that limelight will steer to when it sees a target (Degrees)
       */
      public double backlashSteeringOffsetDegs = 0.0;
      /**
       * Desired area of target from Limelight vision (% of target from limelight vision)
       */
      public double areaThresholdPercentage = 1.75;
      /**
       * Proportional value for PID control for distance assist
       */
      public double kP = 0.225;
    }

    public static enum TurnDirection {
        CW(-1), CCW(+1);

        public final int sign;

        private TurnDirection(int sign) {
            this.sign = sign;
        }
    }

    public static enum Transform {
        BOTPOSE, BOTPOSE_WPIBLUE, BOTPOSE_WPIRED, CAMERAPOSE_TARGETSPACE, TARGETPOSE_CAMERASPACE, TARGETPOSE_ROBOTSPACE, BOTPOSE_TARGETSPACE;
    }
}
