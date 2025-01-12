package org.carlmontrobotics.lib199;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.SparkMax;

import org.mockito.invocation.InvocationOnMock;

public class DummySparkMaxAnswer extends REVLibErrorAnswer {

    private static final long serialVersionUID = 2284848703213263465L;

    public static final DummySparkMaxAnswer ANSWER = new DummySparkMaxAnswer();

    public static final SparkMax DUMMY_SPARK_MAX = Mocks.mock(SparkMax.class, ANSWER);

    public static final RelativeEncoder DUMMY_ENCODER = Mocks.mock(RelativeEncoder.class, REVLibErrorAnswer.ANSWER);
    public static final SparkAnalogSensor DUMMY_ANALOG_SENSOR = Mocks.mock(SparkAnalogSensor.class, REVLibErrorAnswer.ANSWER);
    public static final SparkLimitSwitch DUMMY_LIMIT_SWITCH = Mocks.mock(SparkLimitSwitch.class, REVLibErrorAnswer.ANSWER);
    public static final SparkClosedLoopController DUMMY_PID_CONTROLLER = Mocks.mock(SparkClosedLoopController.class, ANSWER);
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
        } else if(returnType == SparkClosedLoopController.class) {
            return DUMMY_PID_CONTROLLER;
        } else if(returnType == MotorType.class) {
            return MotorType.kBrushless;
        } else if(returnType == IdleMode.class) {
            return IdleMode.kBrake;
        } else if(returnType == MAXMotionConfig.MAXMotionPositionMode.class) {
            return MAXMotionConfig.MAXMotionPositionMode.kMAXMotionTrapezoidal;
        } else if(returnType == SparkAbsoluteEncoder.class) {
            return DUMMY_ABSOLUTE_ENCODER;
        }
        return super.answer(invocation);
    }

}