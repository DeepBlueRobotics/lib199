package org.carlmontrobotics.lib199;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;

public enum MotorControllerType {
    SPARK_MAX,
    SPARK_FLEX;
    public static MotorControllerType getMotorControllerType(SparkBase motor){
        return MotorControllerFactory.getControllerType(motor);
    }
    public SparkBaseConfig createConfig(){
        return MotorControllerFactory.createConfig(this);
    }
}
