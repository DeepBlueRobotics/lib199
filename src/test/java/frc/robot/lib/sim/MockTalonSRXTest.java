package frc.robot.lib.sim;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

public class MockTalonSRXTest extends MockPheonixControllerTest {

    @Override
    protected BaseMotorController createController(int portPWM) {
        return MockTalonSRX.createMockTalonSRX(portPWM);
    }
    
}
