package frc.robot.lib.sim;

import java.util.HashMap;

/**
 * Stores basic properties about how to configure the robot simulation
 */
public class BaseSimConfig {
    private static int sensorTimestep = 20;
    private static double defaultWheelDiameter = 1;
    private static HashMap<String, Double> wheelDiameters = new HashMap<>();

    /**
     * Sets the sampling period to be used when enabling Webots sensors
     * @param timestep the sampling period
     * @see #getSensorTimestep()
     */
    protected static void setSensorTimestep(int timestep) {
        sensorTimestep = timestep;
    }

    /**
     * Retrieves the sampling period to be used when enabling Webots sensors
     * @see #setSensorTimestep(int)
     */
    protected static int getSensorTimestep() {
        return sensorTimestep;
    }

    /**
     * Sets the default wheel diameter to be used when a specific one is not set
     * @param diameter the new default wheel diameter
     * @see #setWheelDiameter(String, double)
     * @see #getWheelDiameter(String)
     */
    protected static void setDefaultWheelDiameter(double diameter) {
        defaultWheelDiameter = diameter;
    }

    /**
     * Sets the wheel diameter for a specific motor
     * @param motor the name of the motor
     * @param diameter the diameter of the wheel attached to the specified motor
     * @see #setDefaultWheelDiameter(double)
     * @see #getWheelDiameter(String)
     */
    protected static void setWheelDiameter(String motor, double diameter) {
        wheelDiameters.put(motor, diameter);
    }

    /**
     * Retrieves the wheel diameter for a specific motor
     * @param motor the name of the motor
     * @return the wheel diameter for the specified motor
     * @see #setWheelDiameter(String, double)
     * @see #setDefaultWheelDiameter(double)
     */
    public static double getWheelDiameter(String motor) {
        return wheelDiameters.containsKey(motor) ? wheelDiameters.get(motor) : defaultWheelDiameter;
    }
}