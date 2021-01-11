package frc.robot.lib.sim;

import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.controller.PIDController;

public class MockedCANPIDController extends CANPIDController {
    private CANSparkMax device;
    private PIDController pidController;

    public MockedCANPIDController(CANSparkMax device) {
        super(device);
        pidController = new PIDController(0.0, 0.0, 0.0);
    }

    @Override
    public CANError setReference(double value, ControlType ctrl, int pidSlot, double arbFeedforward) {
        return CANError.kOk;
    }

    @Override
    public CANError setP(double gain, int slotID) {
        pidController.setP(gain);
        return CANError.kOk;
    }

    @Override
    public double getP(int slotID) {
        return pidController.getP();
    }

    @Override
    public CANError setI(double gain, int slotID) {
        pidController.setI(gain);
        return CANError.kOk;
    }

    @Override
    public double getI(int slotID) {
        return pidController.getI();
    }

    @Override
    public CANError setD(double gain, int slotID) {
        pidController.setD(gain);
        return CANError.kOk;
    }

    @Override
    public double getD(int slotID) {
        return pidController.getD();
    }

    @Override
    public CANError setFF(double gain, int slotID) {
        return CANError.kOk;
    }

    @Override
    public CANError setIZone(double IZone, int slotID) {
        return CANError.kOk;
    }

    @Override
    public CANError setOutputRange(double min, double max, int slotID) {
        return CANError.kOk;
    }
}
