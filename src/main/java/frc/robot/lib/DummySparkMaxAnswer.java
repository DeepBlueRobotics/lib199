package frc.robot.lib;

import com.revrobotics.CANSparkMax;
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

    public static final DummySparkMaxAnswer ANSWER = new DummySparkMaxAnswer();

    public static final CANSparkMax DUMMY_SPARK_MAX = Mockito.mock(CANSparkMax.class, ANSWER);

    public static final RelativeEncoder DUMMY_ENCODER = Mockito.mock(RelativeEncoder.class, REVLibErrorAnswer.ANSWER);
    public static final SparkMaxAnalogSensor DUMMY_ANALOG_SENSOR = Mockito.mock(SparkMaxAnalogSensor.class, REVLibErrorAnswer.ANSWER);
    public static final SparkMaxLimitSwitch DUMMY_LIMIT_SWITCH = Mockito.mock(SparkMaxLimitSwitch.class, REVLibErrorAnswer.ANSWER);
    public static final SparkMaxPIDController DUMMY_PID_CONTROLLER = Mockito.mock(SparkMaxPIDController.class, ANSWER);

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
        }
        return super.answer(invocation);
    }

}