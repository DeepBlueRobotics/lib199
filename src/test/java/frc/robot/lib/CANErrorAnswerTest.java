package frc.robot.lib;

import static org.junit.Assert.assertEquals;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;

import org.junit.Test;
import org.mockito.Mockito;

public class CANErrorAnswerTest {

    @Test
    public void testResponses() throws Exception {
        CANEncoder enc = Mockito.mock(CANEncoder.class, new CANErrorAnswer());
        
        // Check that primative types return "null"
        assertEquals(0, enc.getPosition(), 0.01);
        assertEquals(0, enc.getVelocity(), 0.01);

        // Check that CANError functions return CANError.kOk
        assertEquals(CANError.kOk, enc.setInverted(false));
    }

}
