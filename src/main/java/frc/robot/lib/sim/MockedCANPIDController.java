package frc.robot.lib.sim;

import com.revrobotics.CANError;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.controller.PIDController;

public class MockedCANPIDController {
    private PIDController pidController;

    public MockedCANPIDController(MockSparkMax device) {
        pidController = new PIDController(0.0, 0.0, 0.0);
    }

    public CANError setReference(double value, ControlType ctrl, int pidSlot, double arbFeedforward) {
        return CANError.kOk;
    }

    public CANError setP(double gain, int slotID) {
        pidController.setP(gain);
        return CANError.kOk;
    }

    public double getP(int slotID) {
        return pidController.getP();
    }

    public CANError setI(double gain, int slotID) {
        pidController.setI(gain);
        return CANError.kOk;
    }

    public double getI(int slotID) {
        return pidController.getI();
    }

    public CANError setD(double gain, int slotID) {
        pidController.setD(gain);
        return CANError.kOk;
    }

    public double getD(int slotID) {
        return pidController.getD();
    }

    public CANError setFF(double gain, int slotID) {
        return CANError.kOk;
    }

    public CANError setIZone(double IZone, int slotID) {
        return CANError.kOk;
    }

    public CANError setOutputRange(double min, double max, int slotID) {
        return CANError.kOk;
    }
}
