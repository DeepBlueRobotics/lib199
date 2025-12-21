package org.carlmontrobotics.lib199;

public enum SparkMotorType {
    NEO(MotorControllerType.SPARK_MAX),
    NEO550(MotorControllerType.SPARK_MAX),
    VORTEX(MotorControllerType.SPARK_FLEX),
    SOLO_VORTEX(MotorControllerType.SPARK_MAX), //a spark flex with a solo addapter, we don't really use those but I included it here just in case
    NEO_2(MotorControllerType.SPARK_MAX);

    private final MotorControllerType controllerType;

    SparkMotorType(MotorControllerType controllerType) {
        this.controllerType = controllerType;
    }

    public MotorControllerType getControllerType() {
        return controllerType;
    }
}