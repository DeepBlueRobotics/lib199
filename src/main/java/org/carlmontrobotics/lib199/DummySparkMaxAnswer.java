package org.carlmontrobotics.lib199;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import org.mockito.invocation.InvocationOnMock;

@Deprecated
public class DummySparkMaxAnswer extends REVLibErrorAnswer {

    private static final long serialVersionUID = 2284848703213263465L;

    public static final DummySparkMaxAnswer ANSWER = new DummySparkMaxAnswer();

    public static final CANSparkMax DUMMY_SPARK_MAX = Mocks.mock(CANSparkMax.class, ANSWER);

    public static final RelativeEncoder DUMMY_ENCODER = Mocks.mock(RelativeEncoder.class, REVLibErrorAnswer.ANSWER);
    public static final SparkMaxAnalogSensor DUMMY_ANALOG_SENSOR = Mocks.mock(SparkMaxAnalogSensor.class, REVLibErrorAnswer.ANSWER);
    public static final SparkMaxLimitSwitch DUMMY_LIMIT_SWITCH = Mocks.mock(SparkMaxLimitSwitch.class, REVLibErrorAnswer.ANSWER);
    public static final SparkMaxPIDController DUMMY_PID_CONTROLLER = Mocks.mock(SparkMaxPIDController.class, ANSWER);
    public static final SparkMaxAbsoluteEncoder DUMMY_ABSOLUTE_ENCODER = Mocks.mock(SparkMaxAbsoluteEncoder.class, ANSWER);


    @Override
    public Object answer(InvocationOnMock invocation) throws Throwable {
        Class<?> returnType = invocation.getMethod().getReturnType();
        if(returnType == RelativeEncoder.class) {
            return DUMMY_ENCODER;
        } else if(returnType == SparkMaxAnalogSensor.class) {
            return DUMMY_ANALOG_SENSOR;
        } else if(returnType == SparkMaxLimitSwitch.class) {
            return DUMMY_LIMIT_SWITCH;
        } else if(returnType == SparkMaxPIDController.class) {
            return DUMMY_PID_CONTROLLER;
        } else if(returnType == MotorType.class) {
            return MotorType.kBrushless;
        } else if(returnType == IdleMode.class) {
            return IdleMode.kBrake;
        } else if(returnType == AccelStrategy.class) {
            return AccelStrategy.kTrapezoidal;
        } else if(returnType == SparkMaxAbsoluteEncoder.class) {
            return DUMMY_ABSOLUTE_ENCODER;
        }
        return super.answer(invocation);
    }

}