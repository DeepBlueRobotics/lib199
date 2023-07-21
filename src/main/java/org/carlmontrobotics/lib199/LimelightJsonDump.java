package org.carlmontrobotics.lib199;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.annotation.JsonDeserialize;
import com.fasterxml.jackson.databind.util.StdConverter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * A class that represents the JSON dump from the Limelight.
 * 
 * Note: This class was only designed to read JSON dumps from the Limelight. It should not be used for generating a JSON dump.
 */
public class LimelightJsonDump {

    @JsonProperty("pID")
    public int pipelineIndex;
    @JsonProperty("tl")
    public long latencyMs;
    @JsonProperty("ts")
    public long timestampMs;
    @JsonProperty("v")
    @JsonFormat(shape = JsonFormat.Shape.NUMBER)
    public boolean hasValidTargets;
    @JsonProperty("botpose")
    @JsonDeserialize(converter = Pose3dConverter.class)
    public Pose3d robotPose;
    @JsonProperty("Retro")
    public RetroResults[] retroResults;
    @JsonProperty("Fiducial")
    public FiducialResults[] fiducialResults;
    @JsonProperty("Detector")
    public DetectorResults[] detectorResults;
    @JsonProperty("Classifier")
    public ClassifierResults[] classifierResults;

    public static class RetroResults {
        @JsonProperty("pts")
        public double[][] points;
        @JsonProperty("t6c_ts")
        @JsonDeserialize(converter = Pose3dConverter.class)
        public Pose3d cameraPoseInTargetSpace;
        @JsonProperty("t6r_fs")
        @JsonDeserialize(converter = Pose3dConverter.class)
        public Pose3d robotPoseInFieldSpace;
        @JsonProperty("t6r_ts")
        @JsonDeserialize(converter = Pose3dConverter.class)
        public Pose3d robotPoseInTargetSpace;
        @JsonProperty("t6t_cs")
        @JsonDeserialize(converter = Pose3dConverter.class)
        public Pose3d targetPoseInCameraSpace;
        @JsonProperty("t6t_rs")
        @JsonDeserialize(converter = Pose3dConverter.class)
        public Pose3d targetPoseInRobotSpace;
        @JsonProperty("ta")
        public double targetArea;
        @JsonProperty("tx")
        public double centerTxDeg;
        @JsonProperty("txp")
        public double centerTxPx;
        @JsonProperty("ty")
        public double centerTyDeg;
        @JsonProperty("typ")
        public double centerTyPx;
    }

    @JsonIgnoreProperties("skew")
    public static class FiducialResults {
        @JsonProperty("fID")
        public int tagId;
        @JsonProperty("fam")
        public String tagFamily;
        @JsonProperty("pts")
        public double[][] points;
        @JsonProperty("t6c_ts")
        @JsonDeserialize(converter = Pose3dConverter.class)
        public Pose3d cameraPoseInTargetSpace;
        @JsonProperty("t6r_fs")
        @JsonDeserialize(converter = Pose3dConverter.class)
        public Pose3d robotPoseInFieldSpace;
        @JsonProperty("t6r_ts")
        @JsonDeserialize(converter = Pose3dConverter.class)
        public Pose3d robotPoseInTargetSpace;
        @JsonProperty("t6t_cs")
        @JsonDeserialize(converter = Pose3dConverter.class)
        public Pose3d targetPoseInCameraSpace;
        @JsonProperty("t6t_rs")
        @JsonDeserialize(converter = Pose3dConverter.class)
        public Pose3d targetPoseInRobotSpace;
        @JsonProperty("ta")
        public double targetArea;
        @JsonProperty("tx")
        public double centerTxDeg;
        @JsonProperty("txp")
        public double centerTxPx;
        @JsonProperty("ty")
        public double centerTyDeg;
        @JsonProperty("typ")
        public double centerTyPx;
    }

    public static class DetectorResults {
        @JsonProperty("class")
        public String className;
        @JsonProperty("classID")
        public int classId;
        @JsonProperty("conf")
        public double confidence;
        @JsonProperty("pts")
        public double[][] points;
        @JsonProperty("ta")
        public double targetArea;
        @JsonProperty("tx")
        public double centerTxDeg;
        @JsonProperty("txp")
        public double centerTxPx;
        @JsonProperty("ty")
        public double centerTyDeg;
        @JsonProperty("typ")
        public double centerTyPx;
    }

    public static class ClassifierResults {
        @JsonProperty("class")
        public String className;
        @JsonProperty("classID")
        public int classId;
        @JsonProperty("conf")
        public double confidence;
    }

    public static class Pose3dConverter extends StdConverter<double[], Pose3d> {
        @Override
        public Pose3d convert(double[] value) {
            return value.length == 0 ? null : new Pose3d(value[0], value[1], value[2],
                new Rotation3d(Math.toRadians(value[3]), Math.toRadians(value[4]), Math.toRadians(value[5])));
        }
    }
}
