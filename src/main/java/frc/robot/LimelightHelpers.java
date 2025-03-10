//LimelightHelpers v1.11

package frc.robot;

import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.Map;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.util.concurrent.ConcurrentHashMap;

public class LimelightHelpers {
        private static final Map<String, DoubleArrayEntry> arrayEntries = new ConcurrentHashMap<>();

        static class LimelightResults {
                String error;

                @JsonProperty("pID")
                double pipelineID;

                @JsonProperty("botpose_wpired")
                double[] poseRed;

                @JsonProperty("botpose_wpiblue")
                double[] poseBlue;

                @JsonProperty("botpose_tagcount")
                double tagCount;

                @JsonProperty("t6c_rs")
                double[] cameraOffset;

                Pose2d getRedPoseEstimate() {
                        return toPose2D(poseRed);
                }

                Pose2d getBluePoseEstimate() {
                        return toPose2D(poseBlue);
                }

                LimelightResults() {
                        poseRed = new double[6];
                        poseBlue = new double[6];
                        cameraOffset = new double[6];
                }
        }

        static class RawFiducial {
                int id = 0;
                double txnc = 0;
                double tync = 0;
                double ta = 0;
                double distToCamera = 0;
                double distToRobot = 0;
                double ambiguity = 0;

                RawFiducial(int id, double txnc, double tync, double ta, double distToCamera, double distToRobot, double ambiguity) {
                        this.id = id;
                        this.txnc = txnc;
                        this.tync = tync;
                        this.ta = ta;
                        this.distToCamera = distToCamera;
                        this.distToRobot = distToRobot;
                        this.ambiguity = ambiguity;
                }
        }

        static ObjectMapper mapper;
        static boolean profileJSON = false;

        static final String getName(String name) {
                if (name == "" || name == null) {
                        return "limelight";
                }

                return name;
        }

        static Pose2d toPose2D(double[] inData){
                if (inData.length < 6) {
                        return new Pose2d();
                }

                return new Pose2d(
                        new Translation2d(inData[0], inData[1]),
                        new Rotation2d(Units.degreesToRadians(inData[5]))
                );
        }

        public static NetworkTable getNT(String tableName) {
                return NetworkTableInstance.getDefault().getTable(getName(tableName));
        }

        public static void flushNT() {
                NetworkTableInstance.getDefault().flush();
        }

        public static NetworkTableEntry getNTEntry(String tableName, String entryName) {
                return getNT(tableName).getEntry(entryName);
        }

        public static DoubleArrayEntry getArrayEntry(String tableName, String entryName) {
                String key = tableName + "/" + entryName;

                return arrayEntries.computeIfAbsent(key, function -> {
                        NetworkTable table = getNT(tableName);
                        return table.getDoubleArrayTopic(entryName).getEntry(new double[0]);
                });
        }

        public static void setNTArray(String tableName, String entryName, double[] val) {
                getNTEntry(tableName, entryName).setDoubleArray(val);
        }

        public static double[] getNTArray(String tableName, String entryName) {
                return getNTEntry(tableName, entryName).getDoubleArray(new double[0]);
        }

        public static String getNTString(String tableName, String entryName) {
                return getNTEntry(tableName, entryName).getString("");
        }

        public static String getJSONDump(String limelightName) {
                return getNTString(limelightName, "json");
        }

        public static Pose2d getBluePoseEstimate(String limelightName) {
                double[] result = getNTArray(limelightName, "botpose_wpiblue");
                return toPose2D(result);
        }

        public static Pose2d getRedPoseEstimate(String limelightName) {
                double[] result = getNTArray(limelightName, "botpose_wpired");
                return toPose2D(result);
        }
}