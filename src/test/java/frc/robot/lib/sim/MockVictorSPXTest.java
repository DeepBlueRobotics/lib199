package frc.robot.lib.sim;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

public class MockVictorSPXTest extends MockPheonixControllerTest {

    @Override
    protected BaseMotorController createController(int portPWM) {
        return MockVictorSPX.createMockVictorSPX(portPWM);
    }
    
}
