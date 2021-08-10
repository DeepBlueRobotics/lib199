package frc.robot.lib.sim;

import com.revrobotics.CANError;

import edu.wpi.first.wpilibj.controller.PIDController;

public class MockedCANPIDController {
    private PIDController pidController;

    public MockedCANPIDController() {
        pidController = new PIDController(0.0, 0.0, 0.0);
    }

    public CANError setP(double gain) {
        pidController.setP(gain);
        return CANError.kOk;
    }

    public CANError setP(double gain, int slotID) {
        return setP(gain);
    }

    public double getP() {
        return pidController.getP();
    }

    public double getP(int slotID) {
        return getP();
    }

    public CANError setI(double gain) {
        pidController.setI(gain);
        return CANError.kOk;
    }

    public CANError setI(double gain, int slotID) {
        return setI(gain);
    }

    public double getI() {
        return pidController.getI();
    }

    public double getI(int slotID) {
        return getI();
    }

    public CANError setD(double gain) {
        pidController.setD(gain);
        return CANError.kOk;
    }

    public CANError setD(double gain, int slotID) {
        return setD(gain);
    }

    public double getD() {
        return pidController.getD();
    }

    public double getD(int slotID) {
        return getD();
    }
}
