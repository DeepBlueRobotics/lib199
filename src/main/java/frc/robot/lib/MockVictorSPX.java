package frc.robot.lib;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.VictorSP;

public class MockVictorSPX extends MockPhoenixController {
    public MockVictorSPX(int portPWM) {
        super(portPWM);
        motorPWM = new VictorSP(portPWM);
    }

    public static WPI_VictorSPX createMockTalonSRX(int portPWM) {
        return Mocks.createMock(WPI_VictorSPX.class, new MockVictorSPX(portPWM));
    }
}
