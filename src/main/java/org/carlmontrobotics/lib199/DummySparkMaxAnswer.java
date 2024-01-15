package org.carlmontrobotics.lib199;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.AccelStrategy;

import org.mockito.invocation.InvocationOnMock;

public class DummySparkMaxAnswer extends REVLibErrorAnswer {

    private static final long serialVersionUID = 2284848703213263465L;

    public static final DummySparkMaxAnswer ANSWER = new DummySparkMaxAnswer();

    public static final CANSparkMax DUMMY_SPARK_MAX = Mocks.mock(CANSparkMax.class, ANSWER);

    public static final RelativeEncoder DUMMY_ENCODER = Mocks.mock(RelativeEncoder.class, REVLibErrorAnswer.ANSWER);
    public static final SparkAnalogSensor DUMMY_ANALOG_SENSOR = Mocks.mock(SparkAnalogSensor.class, REVLibErrorAnswer.ANSWER);
    public static final SparkLimitSwitch DUMMY_LIMIT_SWITCH = Mocks.mock(SparkLimitSwitch.class, REVLibErrorAnswer.ANSWER);
    public static final SparkPIDController DUMMY_PID_CONTROLLER = Mocks.mock(SparkPIDController.class, ANSWER);
    public static final SparkAbsoluteEncoder DUMMY_ABSOLUTE_ENCODER = Mocks.mock(SparkAbsoluteEncoder.class, ANSWER);


    @Override
    public Object answer(InvocationOnMock invocation) throws Throwable {
        Class<?> returnType = invocation.getMethod().getReturnType();
        if(returnType == RelativeEncoder.class) {
            return DUMMY_ENCODER;
        } else if(returnType == SparkAnalogSensor.class) {
            return DUMMY_ANALOG_SENSOR;
        } else if(returnType == SparkLimitSwitch.class) {
            return DUMMY_LIMIT_SWITCH;
        } else if(returnType == SparkPIDController.class) {
            return DUMMY_PID_CONTROLLER;
        } else if(returnType == MotorType.class) {
            return MotorType.kBrushless;
        } else if(returnType == IdleMode.class) {
            return IdleMode.kBrake;
        } else if(returnType == AccelStrategy.class) {
            return AccelStrategy.kTrapezoidal;
        } else if(returnType == SparkAbsoluteEncoder.class) {
            return DUMMY_ABSOLUTE_ENCODER;
        }
        return super.answer(invocation);
    }

}