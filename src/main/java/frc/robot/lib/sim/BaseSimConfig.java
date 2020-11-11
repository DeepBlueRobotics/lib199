package frc.robot.lib.sim;

import java.util.HashMap;

public class BaseSimConfig {
    private static int sensorTimestep = 20;
    private static double defaultWheelDiameter = 1;
    private static HashMap<String, Double> wheelDiameters = new HashMap<>();

    protected static void setDefaultWheelDiameter(double diameter) {
        defaultWheelDiameter = diameter;
    }

    protected static void setSensorTimestep(int timestep) {
        sensorTimestep = timestep;
    }

    protected static int getSensorTimestep() {
        return sensorTimestep;
    }

    protected static void setWheelDiameter(String motor, double diameter) {
        wheelDiameters.put(motor, diameter);
    }

    public static double getWheelDiameter(String motor) {
        return wheelDiameters.containsKey(motor) ? wheelDiameters.get(motor) : defaultWheelDiameter;
    }
}