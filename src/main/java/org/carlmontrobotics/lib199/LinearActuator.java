package org.carlmontrobotics.lib199;

import edu.wpi.first.wpilibj.PWM;

public class LinearActuator extends PWM {

    private final int minLength, maxLength;

    public LinearActuator(int channel, int minLength, int maxLength) {
        super(channel);
        this.minLength = minLength;
        this.maxLength = maxLength;
        // Our linear actators accept a pulse between 1ms and 2ms
        setBoundsMicroseconds(2, 0, 0, 0, 1);
    }

    public void set(double value) {
        setPosition(value);
    }

    public void setLength(double length) {
        // Get the length as a percentage of maxLength
        set((length - minLength) / (maxLength - minLength));
    }

    public double getLength() {
        return get() * (maxLength - minLength) + minLength;
    }

    public double get() {
        return getPosition();
    }

}
