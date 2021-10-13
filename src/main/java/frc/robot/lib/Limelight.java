/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight {

  public enum Mode {
    DIST, STEER, TARGET
  }

  public final Config config = new Config();
  public final DebugInfo debugInfo = new DebugInfo();
  /*
   * http://docs.limelightvision.io/en/latest/networktables_api.html tv = Whether
   * the limelight has any valid targets (0 or 1) tx = Horizontal Offset From
   * Crosshair To Target (-27 degrees to 27 degrees) ty = Vertical Offset From
   * Crosshair To Target (-20.5 degrees to 20.5 degrees) ta = Target Area (0% of
   * image to 100% of image) There are more values we could be using. Check the
   * documentation.
   */
  private double tv, txDeg, tyDeg, ta;
  private boolean stopSteer = false;
  // Mounting angle is the angle of the limelight (angled up = +, angled down = -)
  private double mountingAngleDeg;

  private double prev_txDeg = 1.0;
  private double prevHeading = 0;

  private PIDController pidController;
  private boolean newPIDLoop = false;

  public Limelight() {
    this("limelight");
  }

  public Limelight(String ntName) {
    config.ntName = ntName;

    double[] pidValues = config.pidValues;
    pidController = new PIDController(pidValues[0], pidValues[1], pidValues[2], 1.0 / 90.0);
    pidController.setSetpoint(0);
    pidController.setTolerance(config.tolerance);
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
    // hypotenuse of height difference and depth difference (ignores left & right
    // difference) between limelight and target
    double hypotenuse = Math.sqrt(forward * forward + diff * diff);
    // left and right distance b/t target and limelight (only x difference, does not
    // inlcude height or depth)
    double strafe = Math.tan(txDeg / 180 * Math.PI) * hypotenuse;
    SmartDashboard.putNumber("distance to ball", forward);
    return new double[] { forward, strafe };
  }

  // Adjusts the distance between a vision target and the robot. Uses basic PID
  // with the ty value from the network table.
  public double distanceAssist() {
    tv = NetworkTableInstance.getDefault().getTable(config.ntName).getEntry("tv").getDouble(0.0);
    ta = NetworkTableInstance.getDefault().getTable(config.ntName).getEntry("ta").getDouble(0.0);
    SmartDashboard.putNumber("Crosshair Vertical Offset", tyDeg);
    double adjustment = 0.0;
    double area_threshold = 1.75;
    double Kp = 0.225;

    if (tv == 1.0) {
      adjustment = (area_threshold - ta) * Kp;
    }
    adjustment = Math.signum(adjustment) * Math.min(Math.abs(adjustment), config.maxAdjustment);
    return adjustment;
  }

  // Adjusts the angle facing a vision target. Uses basic PID with the tx value
  // from the network table.
  public double steeringAssist(double heading) {
    tv = NetworkTableInstance.getDefault().getTable(config.ntName).getEntry("tv").getDouble(0.0);
    txDeg = NetworkTableInstance.getDefault().getTable(config.ntName).getEntry("tx").getDouble(0.0);
    ta = NetworkTableInstance.getDefault().getTable(config.ntName).getEntry("ta").getDouble(0.0);
    SmartDashboard.putNumber("Crosshair Horizontal Offset", txDeg);
    SmartDashboard.putNumber("Found Vision Target", tv);
    SmartDashboard.putNumber("Prev_tx", prev_txDeg);
    txDeg = Double.isNaN(txDeg) ? 0 : txDeg;
    double[] pidValues = config.pidValues;
    pidController.setPID(pidValues[0], pidValues[1], pidValues[2]);
    pidController.setTolerance(config.tolerance);
    double adjustment = 0.0;

    if (tv == 1.0 && !stopSteer) {
      if (ta > config.areaThreshold) {
        adjustment = pidController.calculate(txDeg);
        prev_txDeg = txDeg;

        if (!newPIDLoop) {
          newPIDLoop = true;
          pidController.setSetpoint(Math.signum(prev_txDeg) * config.backlashOffset);
        }
      }
    } else {
      newPIDLoop = false;
      pidController.reset();
      adjustment = Math.copySign(config.steeringFactor, prev_txDeg);
    }

    if (Math.abs(txDeg) < 1.0 && Math.abs(prev_txDeg) < 1.0 && Math.abs(heading - prevHeading) < 1)
      stopSteer = true;
    else
      stopSteer = false;
    if (stopSteer && tv == 1.0) {
      adjustment = 0;
    }
    prevHeading = heading;

    SmartDashboard.putBoolean("Stop Auto Steering", stopSteer);

    adjustment = Math.copySign(Math.min(Math.abs(adjustment), config.maxAdjustment), txDeg);
    SmartDashboard.putNumber("Adjustment", adjustment);
    return adjustment;
  }

  public boolean isAligned() {
    return pidController.atSetpoint();
  }

  // Combination of distance assist and steering assist
  public double[] autoTarget(double heading) {
    double dist_assist = distanceAssist();
    double steer_assist = steeringAssist(heading);
    double[] params = { dist_assist + steer_assist, dist_assist - steer_assist };
    return params;
  }

  public PIDController getPIDController() {
    return pidController;
  }

  public static class Config {
    public String ntName = "limelight";
    public double[] pidValues = { 0.01, 0.03, 0 };
    public double tolerance = 0.01;
    public double steeringFactor = 0.25;
    public double areaThreshold = 0.02;
    public double maxAdjustment = 1.0;
    public double backlashOffset = 0.0;
  }

  public static class DebugInfo {
  }
}
