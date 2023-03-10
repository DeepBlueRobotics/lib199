package org.carlmontrobotics;

public class MotorConfig {

    public static final MotorConfig NEO = new MotorConfig(70, 40);
    public static final MotorConfig NEO_550 = new MotorConfig(40, 20);

    public final int temperatureLimit, currentLimit;

    public MotorConfig(int temperatureLimit, int currentLimit) {
        this.temperatureLimit = temperatureLimit;
        this.currentLimit = currentLimit;
    }

}
