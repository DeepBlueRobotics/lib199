package org.carlmontrobotics.lib199.sim;

import static org.junit.Assert.assertEquals;

import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.function.Supplier;

import com.revrobotics.REVLibError;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import org.carlmontrobotics.lib199.Mocks;
import org.carlmontrobotics.lib199.REVLibErrorAnswer;
import org.junit.Test;

public class MockedSparkMaxPIDControllerTest {
    
    @Test
    public void testResponses() {
        MockSparkMax mockSparkMax = new MockSparkMax(0, MotorType.kBrushless);
        SparkPIDController mock = Mocks.createMock(SparkPIDController.class, new MockedSparkMaxPIDController(mockSparkMax), new REVLibErrorAnswer());
        assertSlotValueUpdate(mock::setP, mock::setP, mock::getP, mock::getP);
        assertSlotValueUpdate(mock::setI, mock::setI, mock::getI, mock::getI);
        assertSlotValueUpdate(mock::setD, mock::setD, mock::getD, mock::getD);
    }

    private void assertSlotValueUpdate(Function<Double, REVLibError> setFunc, BiFunction<Double, Integer, REVLibError> slotSetFunc, Supplier<Double> getFunc, Function<Integer, Double> slotGetFunc) {
        assertSlotValueUpdate(setFunc, getFunc);
        assertSlotValueUpdate(v -> slotSetFunc.apply(v, 0), () -> slotGetFunc.apply(0));
        assertSlotValueUpdate(v -> slotSetFunc.apply(v, 1), () -> slotGetFunc.apply(1));
        assertSlotValueUpdate(v -> slotSetFunc.apply(v, 2), () -> slotGetFunc.apply(2));
    }

    private void assertSlotValueUpdate(Function<Double, REVLibError> setFunc, Supplier<Double> getFunc) {
        assertEquals(REVLibError.kOk, setFunc.apply(0.0));
        assertEquals(0.0, getFunc.get(), 0.01);
        assertEquals(REVLibError.kOk, setFunc.apply(1.0));
        assertEquals(1.0, getFunc.get(), 0.01);
        assertEquals(REVLibError.kOk, setFunc.apply(0.5));
        assertEquals(0.5, getFunc.get(), 0.01);
        assertEquals(REVLibError.kOk, setFunc.apply(-0.5));
        assertEquals(-0.5, getFunc.get(), 0.01);
        assertEquals(REVLibError.kOk, setFunc.apply(-1.0));
        assertEquals(-1.0, getFunc.get(), 0.01);
    }
}
