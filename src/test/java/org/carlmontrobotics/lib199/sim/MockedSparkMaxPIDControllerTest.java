package org.carlmontrobotics.lib199.sim;

import static org.junit.Assert.assertEquals;

import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.function.Supplier;

import com.revrobotics.REVLibError;
import com.revrobotics.SparkPIDController;

import org.carlmontrobotics.lib199.Mocks;
import org.carlmontrobotics.lib199.REVLibErrorAnswer;
import org.junit.Test;

public class MockedSparkMaxPIDControllerTest {
    
    @Test
    public void testResponses() {
        SparkPIDController mock = Mocks.createMock(SparkPIDController.class, new MockedSparkMaxPIDController(), new REVLibErrorAnswer());
        assertSlotValueUpdate(mock::setP, mock::setP, mock::getP, mock::getP);
        assertSlotValueUpdate(mock::setI, mock::setI, mock::getI, mock::getI);
        assertSlotValueUpdate(mock::setD, mock::setD, mock::getD, mock::getD);
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
