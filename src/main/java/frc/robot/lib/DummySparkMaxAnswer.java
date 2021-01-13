package frc.robot.lib;

import com.revrobotics.CANAnalog;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.mockito.Mockito;
import org.mockito.invocation.InvocationOnMock;

public class DummySparkMaxAnswer extends CANErrorAnswer {

    private static final long serialVersionUID = 2284848703213263465L;

    @Override
    public Object answer(InvocationOnMock invocation) throws Throwable {
        Class<?> returnType = invocation.getMethod().getReturnType();
        if(returnType == CANDigitalInput.class) {
            return Mockito.mock(returnType, Mockito.RETURNS_SMART_NULLS);
        } else if(returnType == CANPIDController.class) {
            return Mockito.mock(CANPIDController.class, new CANErrorAnswer() {
                private static final long serialVersionUID = 558452215206948125L;
                @Override
                public Object answer(InvocationOnMock invocation) throws Throwable {
                    if(invocation.getMethod().getReturnType() == AccelStrategy.class) {
                        return AccelStrategy.kTrapezoidal;
                    }
                    return super.answer(invocation);
                }
            });
        } else if(returnType == MotorType.class) {
            return MotorType.kBrushless;
        } else if(returnType == IdleMode.class) {
            return IdleMode.kBrake;
        } else if(returnType == CANEncoder.class || returnType == CANAnalog.class) {
            return Mockito.mock(returnType, new CANErrorAnswer());
        }
        return super.answer(invocation);
    }

}