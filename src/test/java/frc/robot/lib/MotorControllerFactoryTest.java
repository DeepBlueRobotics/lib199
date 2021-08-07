package frc.robot.lib;

import static org.junit.Assert.assertEquals;

import org.junit.ClassRule;
import org.junit.Rule;
import org.junit.Test;

import frc.robot.lib.testUtils.ErrStreamTest;
import frc.robot.lib.testUtils.SimDeviceTestRule;

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
        MotorControllerFactory.createSparkMax(2);
        assertEquals(0, errStream.toByteArray().length);
    }

}
