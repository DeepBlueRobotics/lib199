package org.carlmontrobotics.lib199;

public class MotorConfig {

    public static final MotorConfig NEO = new MotorConfig(70, 40);
    public static final MotorConfig NEO_550 = new MotorConfig(40, 20);

    public final int temperatureLimitCelsius, currentLimitAmps;

    public MotorConfig(int temperatureLimitCelsius, int currentLimitAmps) {
        this.temperatureLimitCelsius = temperatureLimitCelsius;
        this.currentLimitAmps = currentLimitAmps;
    }

}
