package frc.robot.lib.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import org.junit.ClassRule;
import org.junit.Rule;
import org.junit.Test;
import org.mockito.Mockito;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import frc.robot.lib.Mocks;
import frc.robot.lib.testUtils.SafelyClosable;
import frc.robot.lib.testUtils.SimDeviceTestRule;

public abstract class MockPheonixControllerTest {
    
    @ClassRule
    public static SimDeviceTestRule.Class simClassRule = new SimDeviceTestRule.Class();
    @Rule
    public SimDeviceTestRule.Test simTestRule = new SimDeviceTestRule.Test();

    protected abstract BaseMotorController createController(int portPWM);

    private SafelyClosable createSafeController(int portPWM) {
        BaseMotorController controller = createController(portPWM);
        // Turn the controller into a SafelyClosable that calls the defined close() method (if available)
        return (SafelyClosable)Mocks.createMock(
        // Extend the runtime class type (or superclass if mocked)
            (Class<?>)(Mockito.mockingDetails(controller).isMock() ? controller.getClass().getSuperclass() : controller.getClass()),
        // Implement all methods with the implementation defined in controller
            controller,
        // Return null for any methods not defined in the implementation (none of them)
            false,
        // Implement SafelyClosable
            SafelyClosable.class);
    }

    @Test
    public void testDeviceId() {
        assertTestDeviceId(0);
        assertTestDeviceId(1);
        assertTestDeviceId(2);
    }

    private void assertTestDeviceId(int port) {
        PWMSim sim = new PWMSim(port);
        sim.resetData();
        assertFalse(sim.getInitialized());
        try(SafelyClosable controller = createSafeController(port)) {
            BaseMotorController motor = (BaseMotorController)controller;
            assertTrue(sim.getInitialized());
            assertEquals(port, motor.getDeviceID());
        }
        sim.resetData();
        assertFalse(sim.getInitialized());
    }

    @Test
    public void testInverted() {
        withControllers((controller, sController, sim) -> {
            controller.setInverted(true);
            assertTrue(controller.getInverted());
            controller.setInverted(false);
            assertFalse(controller.getInverted());
        });
    }

    @Test
    public void testSet() {
        withControllers((controller, sController, sim) -> {
            assertSet(1, sController, sim);
            assertSet(0.5, sController, sim);
            assertSet(0, sController, sim);
            assertSet(-0.5, sController, sim);
            assertSet(-1, sController, sim);
        });
    }

    private void assertSet(double speed, MotorController controller, PWMSim sim) {
        assertSet(speed, controller, controller, sim);
    }

    private void assertSet(double speed, MotorController setController, MotorController getController, PWMSim sim) {
        setController.set(speed);
        assertEquals(speed, getController.get(), 0.01);
        assertEquals(speed, sim.getSpeed(), 0.01);
    }

    @Test
    public void testFollow() {
        withControllers((mController, msController, mSim) ->
            withController(mController.getDeviceID()+1, (sController, ssController, sSim) -> {
                sController.follow(mController);
                assertSet(1, msController, ssController, sSim);
                assertSet(0.5, msController, ssController, sSim);
                assertSet(0, msController, ssController, sSim);
                assertSet(-0.5, msController, ssController, sSim);
                assertSet(-1, msController, ssController, sSim);
            })
        );
    }

    private void withControllers(ControllerTest func) {
        withController(0, func);
        withController(1, func);
        withController(2, func);
    }

    private void withController(int port, ControllerTest func) {
        try(SafelyClosable controller = createSafeController(port)) {
            PWMSim sim = new PWMSim(port);
            assertTrue(sim.getInitialized());
            func.test((BaseMotorController)controller, (MotorController)controller, sim);
        }
    }

    private interface ControllerTest {
        public void test(BaseMotorController controller, MotorController sController, PWMSim sim);
    }

}
