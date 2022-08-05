package org.carlmontrobotics.lib199.sim;

import com.revrobotics.REVLibError;

import edu.wpi.first.math.controller.PIDController;

public class MockedSparkMaxPIDController {
    private PIDController pidController;

    public MockedSparkMaxPIDController() {
        pidController = new PIDController(0.0, 0.0, 0.0);
    }

    public REVLibError setP(double gain) {
        pidController.setP(gain);
        return REVLibError.kOk;
    }

    public REVLibError setP(double gain, int slotID) {
        return setP(gain);
    }

    public double getP() {
        return pidController.getP();
    }

    public double getP(int slotID) {
        return getP();
    }

    public REVLibError setI(double gain) {
        pidController.setI(gain);
        return REVLibError.kOk;
    }

    public REVLibError setI(double gain, int slotID) {
        return setI(gain);
    }

    public double getI() {
        return pidController.getI();
    }

    public double getI(int slotID) {
        return getI();
    }

    public REVLibError setD(double gain) {
        pidController.setD(gain);
        return REVLibError.kOk;
    }

    public REVLibError setD(double gain, int slotID) {
        return setD(gain);
    }

    public double getD() {
        return pidController.getD();
    }

    public double getD(int slotID) {
        return getD();
    }
}
