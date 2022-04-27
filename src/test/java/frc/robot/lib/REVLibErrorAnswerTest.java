package frc.robot.lib;

import static org.junit.Assert.assertEquals;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import org.junit.Test;

public class REVLibErrorAnswerTest {

    @Test
    public void testResponses() throws Exception {
        RelativeEncoder enc = Mocks.mock(RelativeEncoder.class, new REVLibErrorAnswer());
        
        // Check that primative types return "null"
        assertEquals(0, enc.getPosition(), 0.01);
        assertEquals(0, enc.getVelocity(), 0.01);

        // Check that REVLibError functions return REVLibError.kOk
        assertEquals(REVLibError.kOk, enc.setInverted(false));
    }

}
