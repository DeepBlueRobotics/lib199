package frc.robot.lib.sim;

import java.util.ArrayList;
import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;

import edu.wpi.first.wpilibj.PWMSpeedController;

abstract class MockPhoenixController implements AutoCloseable {
    private final int portPWM;
    private boolean isInverted;
    // Assign the CAN port to a PWM port so it works with the simulator. Not a fan of this solution though
    // CAN ports should be separate from PWM ports
    protected PWMSpeedController motorPWM;
    // Since we need to keep a record of all the motor's followers
    protected static HashMap<Integer, ArrayList<PWMSpeedController>> followMap = new HashMap<>();

    public MockPhoenixController(int portPWM) {
        this.portPWM = portPWM;
        isInverted = false;
    }

    public void set(double speed) {
        speed = (getInverted() ? -1.0 : 1.0) * speed;
        motorPWM.set(speed);
        if (followMap.containsKey(getDeviceID())) {
            for (PWMSpeedController motor : followMap.get(getDeviceID())) motor.set(speed);
        }
    }

    public double get() {
        return motorPWM.get();
    }

    public void follow(IMotorController leader) {
        if (!followMap.containsKey(leader.getDeviceID())) {
            ArrayList<PWMSpeedController> arr = new ArrayList<PWMSpeedController>();
            arr.add(motorPWM);
            followMap.put(leader.getDeviceID(), arr);
        } else {
            followMap.get(leader.getDeviceID()).add(motorPWM);
        }
    }
    
    public void setInverted(boolean invert) { 
        isInverted = invert; 
    }

    public boolean getInverted() { 
        return isInverted; 
	}
	
	public int getDeviceID() { return portPWM; }
    public ControlMode getControlMode() { return ControlMode.PercentOutput; }

    @Override
    public void close() {
        motorPWM.close();
        followMap.values().forEach(followList -> followList.remove(motorPWM));
    }
}
