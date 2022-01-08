package frc.robot.lib;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import org.mockito.Mockito;
import org.mockito.invocation.InvocationOnMock;

public class DummySparkMaxAnswer extends REVLibErrorAnswer {

    private static final long serialVersionUID = 2284848703213263465L;

    @Override
    public Object answer(InvocationOnMock invocation) throws Throwable {
        Class<?> returnType = invocation.getMethod().getReturnType();
        if(returnType == RelativeEncoder.class || returnType == SparkMaxAnalogSensor.class || returnType == SparkMaxAnalogSensor.class || returnType == SparkMaxLimitSwitch.class) {
            return Mockito.mock(returnType, new REVLibErrorAnswer());
        } else if(returnType == SparkMaxPIDController.class) {
            return Mockito.mock(SparkMaxPIDController.class, new REVLibErrorAnswer() {
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
        }
        return super.answer(invocation);
    }

}