package frc.robot.lib;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import com.revrobotics.CANError;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.junit.Test;
import org.mockito.Mockito;

public class DummySparkMaxAnswerTest {
    
    public CANSparkMax createMockedSparkMax() {
        return Mockito.mock(CANSparkMax.class, new DummySparkMaxAnswer());
    }

    public static void assertTestResponses(CANSparkMax spark) {
        // Check device id
        assertEquals(0, spark.getDeviceId());

        // Check get and set methods
        assertEquals(0, spark.get(), 0.01);
        spark.set(0);

        // Check that all REV specific objects are non-null
        assertNotNull(spark.getAnalog(null));
        assertNotNull(spark.getEncoder());
        assertNotNull(spark.getForwardLimitSwitch(null));
        assertNotNull(spark.getPIDController());

        // Check that all REV specific objects return "null" values
        assertEquals(0, spark.getEncoder().getPosition(), 0.01);
        assertEquals(0, spark.getEncoder().getVelocity(), 0.01);
        assertEquals(IdleMode.kBrake, spark.getIdleMode());
        assertEquals(MotorType.kBrushless, spark.getMotorType());
        assertEquals(AccelStrategy.kTrapezoidal, spark.getPIDController().getSmartMotionAccelStrategy(0));
        assertEquals(CANError.kOk, spark.getLastError());
        assertEquals(CANError.kOk, spark.getPIDController().setP(0, 0));
        assertEquals(CANError.kOk, spark.getEncoder().setAverageDepth(0));
        assertEquals(CANError.kOk, spark.getAnalog(null).setInverted(false));
    }

    @Test
    public void testResponses() {
        assertTestResponses(createMockedSparkMax());
    }

}
