package org.carlmontrobotics.lib199;

import static org.junit.Assert.assertEquals;

import org.carlmontrobotics.lib199.testUtils.ErrStreamTest;
import org.carlmontrobotics.lib199.testUtils.TestRules;
import org.junit.ClassRule;
import org.junit.Rule;
import org.junit.Test;

public class MotorControllerFactoryTest extends ErrStreamTest {

    @ClassRule
    public static TestRules.InitializeHAL simClassRule = new TestRules.InitializeHAL();
    @Rule
    public TestRules.ResetSimDeviceSimData simTestRule = new TestRules.ResetSimDeviceSimData();

    @Test
    // AutoClosable.close() throws Exception
    public void testCreateNoErrors() throws Exception {
        // Call close to free PWM ports
        ((AutoCloseable)MotorControllerFactory.createTalon(0)).close();
        ((AutoCloseable)MotorControllerFactory.createVictor(1)).close();
        MotorControllerFactory.createSparkMax(2, MotorConfig.NEO);
        assertEquals(0, errStream.toByteArray().length);
    }

}
