package org.carlmontrobotics.lib199;

import com.ctre.phoenix6.StatusCode;

import org.mockito.internal.stubbing.defaultanswers.ReturnsSmartNulls;
import org.mockito.invocation.InvocationOnMock;

public class ErrorCodeAnswer extends ReturnsSmartNulls {

    private static final long serialVersionUID = -4964720958493493935L;

    @Override
    public Object answer(InvocationOnMock invocation) throws Throwable {
        return invocation.getMethod().getReturnType().equals(StatusCode.class) ? StatusCode.OK : super.answer(invocation);
    }
    
}