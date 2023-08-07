package org.carlmontrobotics.lib199;

import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartString {
    private final String userValue;
    private final String name;
    private final StringEntry entry;

    public SmartString(String name, String userValue)
    {
        this.userValue = userValue;
        this.name = name;
        StringTopic smartNumberTopic = new StringTopic(SmartDashboard.getEntry(name).getTopic());
        entry = smartNumberTopic.getEntry(userValue);
        
        reset();
    }

    public void reset()
    {
        set(getValue());
    }
     
    public void set(String userValue) 
    {
        entry.set(userValue);
    }
   
    public String getValue()
    {
        return userValue;
    }

    public String getName()
    {
        return name;
    }

    public StringEntry getEntry()
    {
        return entry;
    }
}
