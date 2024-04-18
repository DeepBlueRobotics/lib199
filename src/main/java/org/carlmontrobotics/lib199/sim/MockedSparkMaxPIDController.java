package org.carlmontrobotics.lib199.sim;

import java.util.Arrays;

import com.revrobotics.CANSparkBase;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimEnum;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.controller.PIDController;

public class MockedSparkMaxPIDController {
    private PIDController pidController;
    private int portNumber;
    SimDevice pidControllerSim;
    SimDouble velocitySim;
    SimEnum controlTypeSim;
    SimDouble arbFFSim;
    SimEnum arbFFUnitsSim;

    public MockedSparkMaxPIDController(int portNumber) {
        this.portNumber = portNumber;
        pidController = new PIDController(0.0, 0.0, 0.0);
        pidControllerSim = SimDevice.create("SparkPIDController", portNumber);

        velocitySim = pidControllerSim.createDouble("velocity", Direction.kBidir, 0);

        String[] controlTypeNames = Arrays.stream(ControlType.values()).map(ControlType::name).toArray(String[]::new);
        controlTypeSim = pidControllerSim.createEnum("controlType", Direction.kBidir, controlTypeNames, ControlType.kDutyCycle.ordinal());

        arbFFSim = pidControllerSim.createDouble("arbFF", Direction.kBidir, 0);

        String[] arbFFUnitNames = Arrays.stream(ArbFFUnits.values()).map(ArbFFUnits::name).toArray(String[]::new);
        arbFFUnitsSim = pidControllerSim.createEnum("arbFFUnits", Direction.kBidir, arbFFUnitNames, ArbFFUnits.kVoltage.ordinal());
    }

    @Deprecated
    public MockedSparkMaxPIDController() {
        this(0);
    }

    public REVLibError setReference​(double value, CANSparkBase.ControlType ctrl, int pidSlot, double arbFeedforward, SparkPIDController.ArbFFUnits arbFFUnits) {
        controlTypeSim.set(ctrl.ordinal());
        arbFFSim.set(arbFeedforward);
        arbFFUnitsSim.set(arbFFUnits.ordinal());
        switch (ctrl) {
            case kVelocity:
                velocitySim.set(value);
                break;
        }
        return REVLibError.kOk;
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
