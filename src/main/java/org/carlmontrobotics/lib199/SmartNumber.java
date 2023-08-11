package org.carlmontrobotics.lib199;


import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SmartNumber {

    private final double userValue;
    private final String name;
    private final DoubleEntry entry;

    public SmartNumber(String name, double userValue) {
        this.userValue = userValue;
        this.name = name;
        DoubleTopic smartNumberTopic = new DoubleTopic(SmartDashboard.getEntry(name).getTopic());
        entry = smartNumberTopic.getEntry(userValue);

        reset();
    }

    public void reset() {
        set(getValue());
    }

    public void set(double userValue) {
        entry.set(userValue);
    }

    public double getValue() {
        return userValue;
    }

    public String getName() {
        return name;
    }

    public DoubleEntry getEntry() {
        return entry;
    }

}