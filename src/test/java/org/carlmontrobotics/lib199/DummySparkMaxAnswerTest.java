package org.carlmontrobotics.lib199;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import org.junit.Test;

public class DummySparkMaxAnswerTest {
    
    public CANSparkMax createMockedSparkMax() {
        return Mocks.mock(CANSparkMax.class, new DummySparkMaxAnswer());
    }

    public static void assertTestResponses(CANSparkMax spark) {
        // Check device id
        assertEquals(0, spark.getDeviceId());

        // Check get and set methods
        assertEquals(0, spark.get(), 0.01);
        spark.set(0);

        // Check that all REV specific objects are non-null
        assertNotNull(spark.getAnalog((Mode) null));
        assertNotNull(spark.getEncoder());
        assertNotNull(spark.getForwardLimitSwitch((Type) null));
        assertNotNull(spark.getPIDController());

        // Check that all REV specific objects return "null" values
        assertEquals(0, spark.getEncoder().getPosition(), 0.01);
        assertEquals(0, spark.getEncoder().getVelocity(), 0.01);
        assertEquals(IdleMode.kBrake, spark.getIdleMode());
        assertEquals(MotorType.kBrushless, spark.getMotorType());
        assertEquals(AccelStrategy.kTrapezoidal, spark.getPIDController().getSmartMotionAccelStrategy(0));
        assertEquals(REVLibError.kOk, spark.getLastError());
        assertEquals(REVLibError.kOk, spark.getPIDController().setP(0, 0));
        assertEquals(REVLibError.kOk, spark.getEncoder().setAverageDepth(0));
        assertEquals(REVLibError.kOk, spark.getAnalog((Mode) null).setInverted(false));
    }

    @Test
    public void testResponses() {
        assertTestResponses(createMockedSparkMax());
    }

}
