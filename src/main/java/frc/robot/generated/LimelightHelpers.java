//LimelightHelpers v1.11 (REQUIRES LLOS 2025.0 OR LATER)

package frc.robot.generated;

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
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.util.concurrent.ConcurrentHashMap;

public class LimelightHelpers {
    private static final Map<String, DoubleArrayEntry> arrayEntries = new ConcurrentHashMap<>();

    public static class LimelightResults {
        String error;

        @JsonProperty("pID")
        public double pipelineID;

        @JsonProperty("botpose_wpired")
        public double[] poseRed;

        @JsonProperty("botpose_wpiblue")
        public double[] poseBlue;

        @JsonProperty("botpose_tagcount")
        public double tagCount;

        @JsonProperty("t6c_rs")
        public double[] cameraOffset;

        public Pose2d getRedPose() {
                return toPose2D(poseRed);
        }
        
            public Pose2d getBluePose() {
                return toPose2D(poseBlue);
        }

        public LimelightResults() {
            poseRed = new double[6];
            poseBlue = new double[6];
            cameraOffset = new double[6];
        }
    }

    public static class RawFiducial {
        public int id = 0;
        public double txnc = 0;
        public double tync = 0;
        public double ta = 0;
        public double distToCamera = 0;
        public double distToRobot = 0;
        public double ambiguity = 0;

        public RawFiducial(int id, double txnc, double tync, double ta, double distToCamera, double distToRobot, double ambiguity) {
            this.id = id;
            this.txnc = txnc;
            this.tync = tync;
            this.ta = ta;
            this.distToCamera = distToCamera;
            this.distToRobot = distToRobot;
            this.ambiguity = ambiguity;
        }
    }

    private static ObjectMapper mapper;
    static boolean profileJSON = false;

    static final String getName(String name) {
        if (name == "" || name == null) {
            return "limelight";
        }

        return name;
    }

    public static Pose2d toPose2D(double[] inData){
        if (inData.length < 6) {
            return new Pose2d();
        }

        Translation2d tran2d = new Translation2d(inData[0], inData[1]);
        Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));

        return new Pose2d(tran2d, r2d);
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

        return arrayEntries.computeIfAbsent(key, k -> {
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

    public static Pose2d getBluePose(String limelightName) {
        double[] result = getNTArray(limelightName, "botpose_wpiblue");
        return toPose2D(result);
    }

    public static Pose2d getRedPose(String limelightName) {
        double[] result = getNTArray(limelightName, "botpose_wpired");
        return toPose2D(result);
    }

    public static void setOffset(String limelightName, double forward, double side, double up, double roll, double pitch, double yaw) {
        double[] entries = new double[6];

        entries[0] = forward;
        entries[1] = side;
        entries[2] = up;
        entries[3] = roll;
        entries[4] = pitch;
        entries[5] = yaw;

        setNTArray(limelightName, "camerapose_robotspace_set", entries);
    }

    public static LimelightResults getResults(String limelightName) {
        LimelightHelpers.LimelightResults results = new LimelightHelpers.LimelightResults();

        if (mapper == null){
            mapper = new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
        }

        try {
            results = mapper.readValue(getJSONDump(limelightName), LimelightResults.class);
        }
        
        catch (JsonProcessingException e) {
            results.error = "lljson error: " + e.getMessage();
        }

        return results;
    }
}