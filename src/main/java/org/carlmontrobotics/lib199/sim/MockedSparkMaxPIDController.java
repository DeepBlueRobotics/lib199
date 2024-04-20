package org.carlmontrobotics.lib199.sim;

import java.util.Arrays;

import com.revrobotics.CANSparkBase;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimEnum;
import edu.wpi.first.hal.SimInt;
import edu.wpi.first.hal.SimLong;
import edu.wpi.first.hal.SimDevice.Direction;

public class MockedSparkMaxPIDController {
    private static class ControlParameters {
        SimDouble pSim, iSim, dSim;

        ControlParameters(SimDevice device, int slot) {
            pSim = device.createDouble(String.format("p[%d]", slot), Direction.kOutput, 0);
            iSim = device.createDouble(String.format("i[%d]", slot), Direction.kOutput, 0);
            dSim = device.createDouble(String.format("d[%d]", slot), Direction.kOutput, 0);
        }
    }
    private ControlParameters[] controlParams = new ControlParameters[4];

    private int portNumber;
    private SimDevice pidControllerSim;
    private SimBoolean isUpdatingReferenceSim;
    private SimLong numUpdatesSim;
    private SimDouble referenceSim;
    private SimEnum controlTypeSim;
    private SimInt slotSim;
    private SimDouble arbFFSim;
    private SimEnum arbFFUnitsSim;

    public MockedSparkMaxPIDController(int portNumber) {
        this.portNumber = portNumber;
        pidControllerSim = SimDevice.create("SparkPIDController", portNumber);
        for (int i : new int[] {0, 1, 2, 3}) {
            controlParams[i] = new ControlParameters(pidControllerSim, i);
        }

        isUpdatingReferenceSim = pidControllerSim.createBoolean("isUpdating", Direction.kOutput, false);
        numUpdatesSim = pidControllerSim.createLong("numUpdates", Direction.kOutput, 0);

        referenceSim = pidControllerSim.createDouble("reference", Direction.kOutput, 0);

        String[] controlTypeNames = Arrays.stream(ControlType.values()).map(ControlType::name).toArray(String[]::new);
        controlTypeSim = pidControllerSim.createEnum("controlType", Direction.kOutput, controlTypeNames, ControlType.kDutyCycle.ordinal());

        slotSim = pidControllerSim.createInt("slot", Direction.kOutput, 0);
        arbFFSim = pidControllerSim.createDouble("arbFF", Direction.kOutput, 0);

        String[] arbFFUnitNames = Arrays.stream(ArbFFUnits.values()).map(ArbFFUnits::name).toArray(String[]::new);
        arbFFUnitsSim = pidControllerSim.createEnum("arbFFUnits", Direction.kOutput, arbFFUnitNames, ArbFFUnits.kVoltage.ordinal());
    }

    @Deprecated
    public MockedSparkMaxPIDController() {
        this(0);
    }

    public REVLibError setReference​(double value, CANSparkBase.ControlType ctrl, int pidSlot, double arbFeedforward, SparkPIDController.ArbFFUnits arbFFUnits) {
        isUpdatingReferenceSim.set(true);
        try {
            referenceSim.set(value);
            controlTypeSim.set(ctrl.ordinal());
            slotSim.set(pidSlot);
            arbFFSim.set(arbFeedforward);
            arbFFUnitsSim.set(arbFFUnits.ordinal());
        } finally {
            numUpdatesSim.set(numUpdatesSim.get()+1);
            isUpdatingReferenceSim.set(false);
        }
        return REVLibError.kOk;
    }

    public REVLibError setP(double gain) {
        return setP(gain, 0);
    }

    public REVLibError setP(double gain, int slotID) {
        controlParams[slotID].pSim.set(gain);
        return REVLibError.kOk;
    }

    public double getP() {
        return getP(0);
    }

    public double getP(int slotID) {
        return controlParams[slotID].pSim.get();
    }

    public REVLibError setI(double gain) {
        return setI(gain, 0);
    }

    public REVLibError setI(double gain, int slotID) {
        controlParams[slotID].iSim.set(gain);
        return REVLibError.kOk;
    }

    public double getI() {
        return getI(0);
    }

    public double getI(int slotID) {
        return controlParams[slotID].iSim.get();
    }

    public REVLibError setD(double gain) {
        return setD(gain, 0);
    }

    public REVLibError setD(double gain, int slotID) {
        controlParams[slotID].dSim.set(gain);
        return REVLibError.kOk;
    }

    public double getD() {
        return getD(0);
    }

    public double getD(int slotID) {
        return controlParams[slotID].dSim.get();
    }
}
