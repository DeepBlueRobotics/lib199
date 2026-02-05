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

        MotorControllerFactory.createSparkMax(2, MotorConfig.NEO);
        MotorControllerFactory.createSparkMax(3, MotorConfig.NEO, MotorControllerFactory.sparkConfig(MotorConfig.NEO));
        
        MotorControllerFactory.createSparkFlex(10);
        MotorControllerFactory.createSparkFlex(10, MotorConfig.NEO_VORTEX, MotorControllerFactory.sparkConfig(MotorConfig.NEO_VORTEX));

        MotorControllerFactory.createSpark(20, MotorConfig.NEO);
        MotorControllerFactory.createSpark(21, MotorConfig.NEO_2);
        MotorControllerFactory.createSpark(22, MotorConfig.NEO_VORTEX);
        MotorControllerFactory.createSpark(23, MotorConfig.NEO, MotorControllerFactory.sparkConfig(MotorConfig.NEO_VORTEX));
        MotorControllerFactory.createSpark(24, MotorConfig.NEO_2, MotorControllerFactory.sparkConfig(MotorConfig.NEO_VORTEX));
        MotorControllerFactory.createSpark(25, MotorConfig.NEO_VORTEX, MotorControllerFactory.sparkConfig(MotorConfig.NEO_VORTEX));
        
        assertEquals(0, errStream.toByteArray().length);
    }

}
