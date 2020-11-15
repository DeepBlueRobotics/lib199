package frc.robot.lib;

import com.revrobotics.CANError;

import org.mockito.internal.stubbing.defaultanswers.ReturnsSmartNulls;
import org.mockito.invocation.InvocationOnMock;

public class CANErrorAnswer extends ReturnsSmartNulls {

    private static final long serialVersionUID = -561160298532167923L;

    @Override
    public Object answer(InvocationOnMock invocation) throws Throwable {
        return invocation.getMethod().getReturnType().equals(CANError.class) ? CANError.kOk : super.answer(invocation);
    }
    
}