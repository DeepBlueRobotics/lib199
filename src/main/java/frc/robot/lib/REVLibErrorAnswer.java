package frc.robot.lib;

import com.revrobotics.REVLibError;

import org.mockito.internal.stubbing.defaultanswers.ReturnsSmartNulls;
import org.mockito.invocation.InvocationOnMock;

public class REVLibErrorAnswer extends ReturnsSmartNulls {

    private static final long serialVersionUID = -561160298532167923L;

    public static final REVLibErrorAnswer ANSWER = new REVLibErrorAnswer();

    @Override
    public Object answer(InvocationOnMock invocation) throws Throwable {
        return invocation.getMethod().getReturnType().equals(REVLibError.class) ? REVLibError.kOk : super.answer(invocation);
    }
    
}