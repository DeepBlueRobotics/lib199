package org.carlmontrobotics.lib199;

public class MotorConfig {

    public static final MotorConfig NEO = new MotorConfig(70, 40, MotorControllerType.SPARK_MAX);
    public static final MotorConfig NEO_550 = new MotorConfig(70, 40, MotorControllerType.SPARK_MAX);
    public static final MotorConfig NEO_2 = new MotorConfig(70, 40, MotorControllerType.SPARK_MAX); //TODO: find the max temp for NEO 2.0

    // The temp limit of 100C for the Vortex is based on the fact that its temp sensors are mounted directly on the
    // windings (which is not the case for the Neo or Neo550, causing them to have very delayed temp readings) and the 
    // fact that the winding enamel will melt at 140C. 
    // See: https://www.chiefdelphi.com/t/rev-robotics-spark-flex-and-neo-vortex/442595/349?u=brettle
    // As a result I think 100C should be safe. I wouldn't increase it past 120. --Dean
    public static final MotorConfig NEO_VORTEX = new MotorConfig(100, 40, MotorControllerType.SPARK_FLEX);
    public static final MotorConfig NEO_SOLO_VORTEX = new MotorConfig(100, 40, MotorControllerType.SPARK_MAX);
    public final int temperatureLimitCelsius, currentLimitAmps;
    public final MotorControllerType controllerType;

    public MotorConfig(int temperatureLimitCelsius, int currentLimitAmps, MotorControllerType controllerType) {
        this.temperatureLimitCelsius = temperatureLimitCelsius;
        this.currentLimitAmps = currentLimitAmps;
        this.controllerType = controllerType;
    }

    public MotorControllerType getControllerType() {
        return controllerType;
    }
}
