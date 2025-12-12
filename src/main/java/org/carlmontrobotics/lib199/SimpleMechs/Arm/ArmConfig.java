package org.carlmontrobotics.lib199.SimpleMechs.Arm;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;


public final class ArmConfig {
    double armRelativeGearReduction, armAbosoluteGearReduction, armPIDTolerance, armMaxVolts;
    int[] armFollowId;
    MotorConfig[] armFollowMotorType; 
    boolean[] armFollowInversed;

    int armMasterMotorId;
    MotorConfig armMasterMotorType; 
    boolean armMasterInversed;
    
    double [] armKPID, armKFeedForward;
    double armKP, armKI, armKD, armKS, armKG, armKV, armKA;
    
    AbsoluteEncoder armMainAbsoluteEncoder;
    RelativeEncoder armMainRelativeEncoder;
    RelativeEncoder armBackupRelativeEncoder;

    ArmFeedforward armFeed;
    PIDController armPID;
    boolean armMainAbsoluteEncoderExists, armMainRelativeEncoderExists, armBackupRelativeEncoderExists, armPIDContinuousInput;
    Enum<?> armStates; 

    SparkBase armMasterMotor;

    



    public ArmConfig(
        double armRelativeGearReduction, double armAbosoluteGearReduction,
        double armMaxVolts,
        int[] armFollowId, MotorConfig[] armFollowMotorType, boolean[] armFollowInversed,
        int armMasterMotorId, MotorConfig armMasterMotorType, boolean armMasterInversed,
        double[] armKPID, double armPIDTolerance, boolean armPIDContinuousInput,
        double[] armKFeedForward,
        AbsoluteEncoder armMainAbsoluteEncoder, int armMotorIDOfBackupRelativeEncoder,
        Object armStates
        ) {
        
        this.armRelativeGearReduction = armRelativeGearReduction;
        this.armAbosoluteGearReduction = armAbosoluteGearReduction;
        this.armFollowId = armFollowId;
        this.armFollowMotorType = armFollowMotorType;
        this.armFollowInversed = armFollowInversed;
        this.armMasterMotorId = armMasterMotorId;
        this.armMasterMotorType = armMasterMotorType;
        this.armMasterInversed = armMasterInversed;

        this.armMainAbsoluteEncoder = armMainAbsoluteEncoder;
        this.armPIDTolerance = armPIDTolerance;
        this.armPIDContinuousInput = armPIDContinuousInput;
        this.armMaxVolts = armMaxVolts;
        this.armKPID = armKPID;
        this.armKFeedForward = armKFeedForward;

        checkRequirements(armStates, armMotorIDOfBackupRelativeEncoder);

    }


    private void checkRequirements(Object armStates, int armMotorOfBackupRelativeEncoder) {
        matchMotors();
        checkMotorConfigs(); //Prevent future issues with new motors
        checkEncoders(armMotorOfBackupRelativeEncoder);
        checkPID();
        checkFeedForward();
        checkGearReduction();
        checkEnum(armStates);
    }

    private void matchMotors() {
        if (armFollowId != null) {
            if (armFollowId.length == armFollowInversed.length
            && armFollowId.length == armFollowMotorType.length) {
                return;
            }
            throw new IllegalArgumentException("Motors must have matching IDs, MotorTypes and Inverse Configs");
        }
        if (armMasterMotorId != -1) {
            if (armMasterMotorType != null) {
                return;
            }
            throw new IllegalArgumentException("Master motor needs a MotorConfig");
        }
        throw new IllegalArgumentException("Master motor needs to exist");
    }

    private void checkMotorConfigs() {
        for (MotorConfig config: armFollowMotorType) {
            if (config != MotorConfig.NEO && config != MotorConfig.NEO_VORTEX && config != MotorConfig.NEO_550) {
                throw new IllegalArgumentException("What is this config??? If null pls change, if not null you're cooked");
            }
        }
        if (armMasterMotorType != MotorConfig.NEO && armMasterMotorType != MotorConfig.NEO_VORTEX && armMasterMotorType != MotorConfig.NEO_550) {
            throw new IllegalArgumentException("What is this config???");
        }
    }

    private void checkEncoders(int armMotorOfBackupRelativeEncoder) {
        if (armMainAbsoluteEncoder != null) {
            armMainAbsoluteEncoderExists = true;
        }

        if (armMasterMotorType == MotorConfig.NEO_VORTEX) {
            SparkFlexConfig flexConfig = new SparkFlexConfig();
            this.armMasterMotor = MotorControllerFactory.createSparkFlex(armMasterMotorId);
            flexConfig.inverted(armMasterInversed)
                      .idleMode(IdleMode.kBrake)
                      .encoder.positionConversionFactor(armRelativeGearReduction * 360) // Degrees
                              .velocityConversionFactor(armRelativeGearReduction/60); //Rotations per second
            armMasterMotor.configure(flexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            armMainRelativeEncoder = armMasterMotor.getEncoder();
        }
        else {
            SparkMaxConfig maxConfig = new SparkMaxConfig();
            armMasterMotor = MotorControllerFactory.createSparkMax(armMasterMotorId, armMasterMotorType);
            maxConfig.inverted(armMasterInversed)
                      .idleMode(IdleMode.kBrake)
                      .encoder.positionConversionFactor(armRelativeGearReduction * 360) // Degrees
                              .velocityConversionFactor(armRelativeGearReduction/60); //Rotations per second
            armMasterMotor.configure(maxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            armMainRelativeEncoder = armMasterMotor.getEncoder();
        }
        if (armMotorOfBackupRelativeEncoder != -1) {
            for (int i = 0; i < armFollowId.length; i++) {
                if (armFollowId[i] == armMotorOfBackupRelativeEncoder) {
                    if (armFollowMotorType[i] == MotorConfig.NEO_VORTEX) {
                        SparkFlexConfig flexConfig = new SparkFlexConfig();
                        SparkFlex dummyFlex = MotorControllerFactory.createSparkFlex(armMasterMotorId);
                        flexConfig.encoder.positionConversionFactor(armRelativeGearReduction * 360) // Degrees
                                          .velocityConversionFactor(armRelativeGearReduction/60); //Rotations per second
                        dummyFlex.configure(flexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
                        armBackupRelativeEncoder = dummyFlex.getEncoder();
                    }
                    else {
                        SparkMaxConfig maxConfig = new SparkMaxConfig();
                        SparkMax dummyMax = MotorControllerFactory.createSparkMax(armMasterMotorId, armMasterMotorType);
                        maxConfig.encoder.positionConversionFactor(armRelativeGearReduction * 360) // Degrees
                                        .velocityConversionFactor(armRelativeGearReduction/60); //Rotations per second
                        dummyMax.configure(maxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
                        armBackupRelativeEncoder = dummyMax.getEncoder();
                    }
                    return;
                }
            }
            throw new IllegalArgumentException("Backup motor id of " + armMotorOfBackupRelativeEncoder + " is not part of provided motors :(");
        }
        else {
            System.out.println("Arm does not have backup encoder, highly recommended");
        }

        
    }

    private void checkPID() {
        if (armKPID.length == 3) {
            armKP = armKPID[0];
            armKI = armKPID[1];
            armKD = armKPID[2];
            armPID = new PIDController(armKP, armKI, armKD);
            armPID.setTolerance(armPIDTolerance);
            if (armPIDContinuousInput) {
                armPID.enableContinuousInput(-180, 180);
            }
            return;
        }
        throw new IllegalArgumentException("Need to have 3 values for PID");
    }

    private void checkFeedForward() {
        if (armKFeedForward.length == 4) {
            armKS = armKFeedForward[0];
            armKG = armKFeedForward[1];
            armKV = armKFeedForward[2];
            armKA = armKFeedForward[3];
            armFeed = new ArmFeedforward(armKS, armKG, armKV, armKA);
            return;
        }
        throw new IllegalArgumentException("Need to have 4 values for Arm FeedForward: Friction, Gravity, Voltage, and Acceleration");
    }   

    private void checkGearReduction() {
        if (armRelativeGearReduction == 0) {
            throw new IllegalArgumentException("Gear reduction for relative encoders cannot be 0");
        }
        if (armMainAbsoluteEncoderExists && armAbosoluteGearReduction == 0) {
            throw new IllegalArgumentException("Absolute gear reduction cannot be 0 if absolute encoder exists");
        }
    }

    private void checkEnum(Object armStates) {
        Class<?> cls = armStates.getClass();
    
        if (!cls.isEnum()) {
            throw new IllegalArgumentException("Expected an enum");
        }
        
        java.lang.reflect.Method getValueMethod;     
        // Ensure a getValue() method exists
        try {
            getValueMethod = cls.getDeclaredMethod("getValue");
        } catch (NoSuchMethodException e) {
            throw new IllegalArgumentException("Enum must have a getValue() method returning the value of each state");
        }
    
        Object[] values = cls.getEnumConstants();
    
        for (Object constant : values) {
            try {
                Object result = getValueMethod.invoke(constant);
    
                if (!(result instanceof Number)) {
                    throw new IllegalArgumentException(
                        "getValue() must return a numeric type: " + constant);
                }
    
                double value = ((Number) result).doubleValue();
    
            } catch (Exception e) {
                throw new RuntimeException("Failed to read enum value", e);
            }
        }
        this.armStates = (Enum<?>) armStates;
    }
    
}