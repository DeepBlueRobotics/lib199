package org.carlmontrobotics.lib199.sim;

import static org.junit.Assert.assertEquals;
import static org.hamcrest.MatcherAssert.*;
import static org.hamcrest.CoreMatchers.*;

import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.function.Supplier;

import com.revrobotics.CANPIDController;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimEnum;
import edu.wpi.first.hal.SimInt;
import edu.wpi.first.hal.SimLong;
import edu.wpi.first.hal.simulation.SimValueCallback;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

import org.carlmontrobotics.lib199.Mocks;
import org.carlmontrobotics.lib199.REVLibErrorAnswer;
import org.carlmontrobotics.lib199.testUtils.TestRules;
import org.junit.ClassRule;
import org.junit.Rule;
import org.junit.Test;

public class MockedSparkMaxPIDControllerTest {

    @ClassRule
    public static TestRules.InitializeHAL classRule = new TestRules.InitializeHAL(); 
    @Rule
    public TestRules.ResetSimDeviceSimData testRule = new TestRules.ResetSimDeviceSimData(); 

    private int numChangesToIsUpdating = 0;

    @Test
    public void testResponses() {
        SparkPIDController mock = Mocks.createMock(SparkPIDController.class, new MockedSparkMaxPIDController(1), 
            new REVLibErrorAnswer(), CANPIDController.class, SparkMaxPIDController.class);
        SimDeviceSim pidControllerSim = new SimDeviceSim("SparkPIDController", 1);
        SimBoolean isUpdatingSim = pidControllerSim.getBoolean("isUpdating");
        SimLong numUpdatesSim = pidControllerSim.getLong("numUpdates");
        SimDouble referenceSim = pidControllerSim.getDouble("reference");
        SimEnum controlTypeSim = pidControllerSim.getEnum("controlType");
        SimInt slotSim = pidControllerSim.getInt("slot");
        SimDouble arbFFSim = pidControllerSim.getDouble("arbFF");
        SimEnum arbFFUnitsSim = pidControllerSim.getEnum("arbFFUnits");
        pidControllerSim.registerValueChangedCallback(isUpdatingSim, new SimValueCallback() {
            @Override
            public void callback(String name, int handle, int direction, HALValue value) {
                numChangesToIsUpdating++;
            }
        }, false);

        assertSlotValueUpdate(pidControllerSim, "p", mock::setP, mock::setP, mock::getP, mock::getP, 
            (slot) -> pidControllerSim.getDouble(String.format("p[%d]", slot)).get());
        assertSlotValueUpdate(pidControllerSim, "i", mock::setI, mock::setI, mock::getI, mock::getI,
            (slot) -> pidControllerSim.getDouble(String.format("i[%d]", slot)).get());
        assertSlotValueUpdate(pidControllerSim, "d", mock::setD, mock::setD, mock::getD, mock::getD,
            (slot) -> pidControllerSim.getDouble(String.format("d[%d]", slot)).get());

        // Defaults
        assertThat(numChangesToIsUpdating, is(0));
        assertThat(isUpdatingSim.get(), is(false));
        assertThat(numUpdatesSim.get(), is(0L));
        assertThat(referenceSim.get(), is(0.0));
        assertThat(controlTypeSim.get(), is(ControlType.kDutyCycle.ordinal()));
        assertThat(slotSim.get(), is(0));
        assertThat(arbFFSim.get(), is(0.0));
        assertThat(arbFFUnitsSim.get(), is(ArbFFUnits.kVoltage.ordinal()));

        // Velocity control
        assertEquals(REVLibError.kOk, mock.setReference(0.5, ControlType.kVelocity, 1, 0.25, ArbFFUnits.kPercentOut));
        assertThat(numChangesToIsUpdating, is(2));
        assertThat(isUpdatingSim.get(), is(false));
        assertThat(numUpdatesSim.get(), is(1L));
        assertThat(referenceSim.get(), is(0.5));
        assertThat(controlTypeSim.get(), is(ControlType.kVelocity.ordinal()));
        assertThat(slotSim.get(), is(1));
        assertThat(arbFFSim.get(), is(0.25));
        assertThat(arbFFUnitsSim.get(), is(ArbFFUnits.kPercentOut.ordinal()));
        assertThat(numUpdatesSim.get(), is(1L));

        // Duty-cycle control (the default)
        assertEquals(REVLibError.kOk, mock.setReference(0.6, ControlType.kDutyCycle, 1, 0.35, ArbFFUnits.kVoltage));
        assertThat(numChangesToIsUpdating, is(4));
        assertThat(isUpdatingSim.get(), is(false));
        assertThat(numUpdatesSim.get(), is(2L));
        assertThat(referenceSim.get(), is(0.6));
        assertThat(controlTypeSim.get(), is(ControlType.kDutyCycle.ordinal()));
        assertThat(slotSim.get(), is(1));
        assertThat(arbFFSim.get(), is(0.35));
        assertThat(arbFFUnitsSim.get(), is(ArbFFUnits.kVoltage.ordinal()));
        assertThat(numUpdatesSim.get(), is(2L));

        // Position control with slot 2
        assertEquals(REVLibError.kOk, mock.setReference(0.5, ControlType.kPosition, 2, 0.25, ArbFFUnits.kPercentOut));
        assertThat(numChangesToIsUpdating, is(6));
        assertThat(isUpdatingSim.get(), is(false));
        assertThat(numUpdatesSim.get(), is(3L));
        assertThat(referenceSim.get(), is(0.5));
        assertThat(controlTypeSim.get(), is(ControlType.kPosition.ordinal()));
        assertThat(slotSim.get(), is(2));
        assertThat(arbFFSim.get(), is(0.25));
        assertThat(arbFFUnitsSim.get(), is(ArbFFUnits.kPercentOut.ordinal()));
        assertThat(numUpdatesSim.get(), is(3L));
    }

    private void assertSlotValueUpdate(SimDeviceSim pidControllerSim, String paramName, Function<Double, REVLibError> setFunc, BiFunction<Double, Integer, REVLibError> slotSetFunc, Supplier<Double> getFunc, Function<Integer, Double> slotGetFunc, Function<Integer, Double> simValueGetFunc) {
        assertEquals(REVLibError.kOk, setFunc.apply(1.0));
        assertThat(getFunc.get(), is(1.0));
        assertEquals(REVLibError.kOk, slotSetFunc.apply(2.0, 0));
        assertEquals(REVLibError.kOk, slotSetFunc.apply(3.0, 1));
        assertEquals(REVLibError.kOk, slotSetFunc.apply(4.0, 2));
        assertEquals(REVLibError.kOk, slotSetFunc.apply(5.0, 3));
        assertThat(getFunc.get(), is(2.0));
        assertThat(slotGetFunc.apply(0), is(2.0));
        assertThat(slotGetFunc.apply(1), is(3.0));
        assertThat(slotGetFunc.apply(2), is(4.0));
        assertThat(slotGetFunc.apply(3), is(5.0));
        assertThat(simValueGetFunc.apply(0), is(2.0));
        assertThat(simValueGetFunc.apply(1), is(3.0));
        assertThat(simValueGetFunc.apply(2), is(4.0));
        assertThat(simValueGetFunc.apply(3), is(5.0));
    }
}
