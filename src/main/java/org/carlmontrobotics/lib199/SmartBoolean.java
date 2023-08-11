package org.carlmontrobotics.lib199;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartBoolean {

    private final boolean userValue;
    private final String name;
    private final BooleanEntry entry;

    public SmartBoolean(String name, boolean userValue) {
        this.userValue = userValue;
        this.name = name;
        BooleanTopic smartNumberTopic = new BooleanTopic(SmartDashboard.getEntry(name).getTopic());
        entry = smartNumberTopic.getEntry(userValue);

        reset();
    }

    public void reset() {
        set(getValue());
    }

    public void set(Boolean userValue) {
        entry.set(userValue);
    }

    public Boolean getValue() {
        return userValue;
    }

    public String getName() {
        return name;
    }

    public BooleanEntry getEntry() {
        return entry;
    }

}
