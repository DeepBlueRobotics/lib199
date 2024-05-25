package org.carlmontrobotics.lib199;

public class MotorConfig {

    public static final MotorConfig NEO = new MotorConfig(70, 40);
    public static final MotorConfig NEO_550 = new MotorConfig(40, 20);

    // The temp limit of 100C for the Vortex is based on the fact that its temp sensors are mounted directly on the
    // windings (which is not the case for the Neo or Neo550, causing them to have very delayed temp readings) and the 
    // fact that the winding enamel will melt at 140C. 
    // See: https://www.chiefdelphi.com/t/rev-robotics-spark-flex-and-neo-vortex/442595/349?u=brettle
    // As a result I think 100C should be safe. I wouldn't increase it past 120. --Dean
    public static final MotorConfig NEO_VORTEX = new MotorConfig(100, 60);

    public final int temperatureLimitCelsius, currentLimitAmps;

    public MotorConfig(int temperatureLimitCelsius, int currentLimitAmps) {
        this.temperatureLimitCelsius = temperatureLimitCelsius;
        this.currentLimitAmps = currentLimitAmps;
    }

}
