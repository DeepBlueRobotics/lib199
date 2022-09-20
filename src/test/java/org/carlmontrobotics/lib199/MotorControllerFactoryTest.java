package org.carlmontrobotics.lib199;

import static org.junit.Assert.assertEquals;

import org.carlmontrobotics.lib199.testUtils.ErrStreamTest;
import org.carlmontrobotics.lib199.testUtils.SimDeviceTestRule;
import org.junit.ClassRule;
import org.junit.Rule;
import org.junit.Test;

public class MotorControllerFactoryTest extends ErrStreamTest {

    @ClassRule
    public static SimDeviceTestRule.Class simClassRule = new SimDeviceTestRule.Class();
    @Rule
    public SimDeviceTestRule.Test simTestRule = new SimDeviceTestRule.Test();

    @Test
    // AutoClosable.close() throws Exception
    public void testCreateNoErrors() throws Exception {
        // Call close to free PWM ports
        ((AutoCloseable)MotorControllerFactory.createTalon(0)).close();
        ((AutoCloseable)MotorControllerFactory.createVictor(1)).close();
        MotorControllerFactory.createSparkMax(2, 40);
        assertEquals(0, errStream.toByteArray().length);
    }

}
