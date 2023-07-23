package org.carlmontrobotics.lib199;


import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SmartNumber
{
    private final double defaultValue;
    private final int handle;
    private final String name;
    private final DoubleEntry entry;
    
   
    


    public SmartNumber(String name, double defaultValue)
    {
        this.defaultValue = defaultValue;
        this.name = name;
        handle = NetworkTablesJNI.getEntry(NetworkTablesJNI.getDefaultInstance(), "SmartDashboard/" + name);
        SmartDashboard.getEntry(name);
        DoubleTopic smartNumberTopic = new DoubleTopic(SmartDashboard.getEntry(name).getTopic());
        entry = smartNumberTopic.getEntry(defaultValue);
        
        reset();


    }
    public void reset()
    {
        set(getDefault());
    }
     
    public void set(double defaultValue) 
    {
        entry.set(defaultValue);
    }
   
    public double getDefault()
    {
        return defaultValue;
    }
   

    
}