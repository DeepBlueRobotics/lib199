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

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimEnum;
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


    @Test
    public void testResponses() {
        SparkPIDController mock = Mocks.createMock(SparkPIDController.class, new MockedSparkMaxPIDController(1), 
            new REVLibErrorAnswer(), CANPIDController.class, SparkMaxPIDController.class);
        assertSlotValueUpdate(mock::setP, mock::setP, mock::getP, mock::getP);
        assertSlotValueUpdate(mock::setI, mock::setI, mock::getI, mock::getI);
        assertSlotValueUpdate(mock::setD, mock::setD, mock::getD, mock::getD);


        SimDeviceSim pidControllerSim = new SimDeviceSim("SparkPIDController", 1);
        SimDouble velocitySim = pidControllerSim.getDouble("velocity");
        SimEnum controlTypeSim = pidControllerSim.getEnum("controlType");
        SimDouble arbFFSim = pidControllerSim.getDouble("arbFF");
        SimEnum arbFFUnitsSim = pidControllerSim.getEnum("arbFFUnits");

        assertThat(velocitySim.get(), is(0.0));
        assertThat(controlTypeSim.get(), is(ControlType.kDutyCycle.ordinal()));
        assertThat(arbFFSim.get(), is(0.0));
        assertThat(arbFFUnitsSim.get(), is(ArbFFUnits.kVoltage.ordinal()));

        assertEquals(REVLibError.kOk, mock.setReference(0.5, ControlType.kVelocity, 1, 0.25, ArbFFUnits.kPercentOut));
        assertThat(velocitySim.get(), is(0.5));
        assertThat(controlTypeSim.get(), is(ControlType.kVelocity.ordinal()));
        assertThat(arbFFSim.get(), is(0.25));
        assertThat(arbFFUnitsSim.get(), is(ArbFFUnits.kPercentOut.ordinal()));
    }

    private void assertSlotValueUpdate(Function<Double, REVLibError> setFunc, BiFunction<Double, Integer, REVLibError> slotSetFunc, Supplier<Double> getFunc, Function<Integer, Double> slotGetFunc) {
        assertSlotValueUpdate(setFunc, getFunc, slotGetFunc);
        assertSlotValueUpdate(v -> slotSetFunc.apply(v, 0), getFunc, slotGetFunc);
        assertSlotValueUpdate(v -> slotSetFunc.apply(v, 1), getFunc, slotGetFunc);
        assertSlotValueUpdate(v -> slotSetFunc.apply(v, 2), getFunc, slotGetFunc);
    }

    private void assertSlotValueUpdate(Function<Double, REVLibError> setFunc, Supplier<Double> getFunc, Function<Integer, Double> slotGetFunc) {
        assertEquals(REVLibError.kOk, setFunc.apply(0.0));
        assertSlotValueGet(0, getFunc, slotGetFunc);
        assertEquals(REVLibError.kOk, setFunc.apply(1.0));
        assertSlotValueGet(1, getFunc, slotGetFunc);
        assertEquals(REVLibError.kOk, setFunc.apply(0.5));
        assertSlotValueGet(0.5, getFunc, slotGetFunc);
        assertEquals(REVLibError.kOk, setFunc.apply(-0.5));
        assertSlotValueGet(-0.5, getFunc, slotGetFunc);
        assertEquals(REVLibError.kOk, setFunc.apply(-1.0));
        assertSlotValueGet(-1, getFunc, slotGetFunc);
    }

    private void assertSlotValueGet(double expected, Supplier<Double> getFunc, Function<Integer, Double> slotGetFunc) {
        assertEquals(expected, getFunc.get(), 0.01);
        assertEquals(expected, slotGetFunc.apply(0), 0.01);
        assertEquals(expected, slotGetFunc.apply(1), 0.01);
        assertEquals(expected, slotGetFunc.apply(2), 0.01);
    }

}
