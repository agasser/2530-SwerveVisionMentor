package frc.robot.subsystems;

import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.CompletableFuture;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.limelight.LimelightResultsMapper;
import frc.robot.limelight.LimelightDetector;
import frc.robot.limelight.LimelightFiducial;
import frc.robot.limelight.LimelightResults;
import frc.robot.limelight.LimelightRetro;
import frc.robot.limelight.VisionTargetInfo;

public class LimelightSubsystem extends SubsystemBase {
    private LimelightResults latestLimelightResults = null;
    boolean profileJSON = false;

    private final NetworkTable limelightNetworkTable;
    private final String networkTableName;

    private boolean takeSnapshot = false;

    private boolean enabled = true;
    private boolean driverMode;
    private double activePipelineId;
    private ObjectMapper mapper;

    public LimelightSubsystem(String networkTableName) {
        limelightNetworkTable = NetworkTableInstance.getDefault().getTable(networkTableName);
        this.networkTableName = networkTableName;

        limelightNetworkTable.getEntry("snapshot").setDouble(0.0);
    }

    // public void addTargetDashboardWidgets(ShuffleboardLayout layout, LimelightHelperFunctions limelightHelperFunctions) {
    //     layout.addBoolean("Target", () -> LimelightHelperFunctions.getLatestRetroTarget(LimelightConstants.limeLightName).isPresent()).withPosition(0, 0);
    //     layout.addDouble("Distance", () -> {
    //     var optResults = LimelightHelperFunctions.getLatestRetroTarget(LimelightConstants.limeLightName);
    //     if (optResults.isPresent()) {
    //         return limelightHelperFunctions.getRobotRelativeTargetInfo(optResults.get()).distance;
    //     }
    //     return 0;
    //     }).withPosition(0, 1);
    //     layout.addDouble("Angle", () -> {
    //     var optResults = getLatestRetroTarget();
    //     if (optResults.isPresent()) {
    //         return limelightHelperFunctions.getRobotRelativeTargetInfo(optResults.get()).angle.getDegrees();
    //     }
    //     return 0;
    //     }).withPosition(0, 2);
    // }

    // public void addDetectorDashboardWidgets(ShuffleboardLayout layout, LimelightHelperFunctions limelightHelperFunctions) {
    //     layout.withProperties(Map.of("Number of columns", 1, "Number of rows", 5));
    //     layout.addBoolean("Target", () -> LimelightHelperFunctions.getLatestDetectorTarget(LimelightConstants.limeLightName).isPresent().withPosition(0, 0);
    //     layout.addDouble("Distance", () -> {
    //     var optResults = getLatestDetectorTarget();
    //     if (optResults.isPresent()) {
    //         return limelightHelperFunctions.getRobotRelativeTargetInfo(optResults.get()).distance;
    //     }
    //     return 0;
    //     }).withPosition(0, 1);
    //     layout.addDouble("Angle", () -> {
    //     var optResults = getLatestDetectorTarget();
    //     if (optResults.isPresent()) {
    //         return limelightHelperFunctions.getRobotRelativeTargetInfo(optResults.get()).angle.getDegrees();
    //     }
    //     return 0;
    //     }).withPosition(0, 2);
    //     layout.addString("Class", () -> {
    //     var optResults = getLatestDetectorTarget();
    //     if (optResults.isPresent()) {
    //         return optResults.get().humanReadableClass;
    //     }
    //     return "";
    //     }).withPosition(0, 3);
    // }

    @Override
    public void periodic() {
        latestLimelightResults = null;
        // Flush NetworkTable to send LED mode and pipeline updates immediately
        var shouldFlush = (limelightNetworkTable.getEntry("ledMode").getDouble(0.0) != (enabled ? 0.0 : 1.0) || 
        limelightNetworkTable.getEntry("pipeline").getDouble(0.0) != activePipelineId);

        limelightNetworkTable.getEntry("ledMode").setDouble(enabled ? 0.0 : 1.0);
        limelightNetworkTable.getEntry("camMode").setDouble(driverMode ? 1.0 : 0.0);
        limelightNetworkTable.getEntry("pipeline").setDouble(activePipelineId);

        if (shouldFlush)  {
            NetworkTableInstance.getDefault().flush();
        }

        if(takeSnapshot) {
            limelightNetworkTable.getEntry("snapshot").setDouble(1.0);
            takeSnapshot = false;
        } else {
            limelightNetworkTable.getEntry("snapshot").setDouble(0.0);
        }
    }

    /**
     * Turns the LEDS off and switches the camera mode to vision processor.
     */
    public void disable() {
        enabled = false;
        driverMode = false;
    }

    /**
     * Sets the LEDS to be controlled by the pipeline and switches the camera mode
     * to vision processor.
     */
    public void enable() {
        enabled = true;
        driverMode = false;
    }

    /**
     * Sets the LEDs to off and switches the camera to driver mode.
     */
    public void driverMode() {
        enabled = false;
        driverMode = true;
    }

    public String getNetworkTableName() {
        return networkTableName;
    }

    public void takeSnapshot() {
        takeSnapshot = true;
    }

    public void setPipelineId(int pipelineId) {
        activePipelineId = pipelineId;
    }

    /**
     * Parses Limelight's JSON results dump into a LimelightResults Object
     */
    public String getJSONDump(String limelightName) {
        return getLimelightNTString(limelightName, "json");
    }
    
    public String getLimelightNTString(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getString("");
    }

    public NetworkTable getLimelightNTTable(String tableName) {
        return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
    }

    public NetworkTableEntry getLimelightNTTableEntry(String tableName, String entryName) {
        return getLimelightNTTable(tableName).getEntry(entryName);
    }

    String sanitizeName(String name) {
        if (name == "" || name == null) {
            return "limelight";
        }
        return name;
    }


    public double getLimelightNTDouble(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getDouble(0.0);
    }

    public double getTX(String limelightName) {
        return getLimelightNTDouble(limelightName, "tx");
    }

    public double getTY(String limelightName) {
        return getLimelightNTDouble(limelightName, "ty");
    }

    public void setLimelightNTDouble(String tableName, String entryName, double val) {
        getLimelightNTTableEntry(tableName, entryName).setDouble(val);
    }

    public void setLimelightNTDoubleArray(String tableName, String entryName, double[] val) {
        getLimelightNTTableEntry(tableName, entryName).setDoubleArray(val);
    }

    public double[] getLimelightNTDoubleArray(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getDoubleArray(new double[0]);
    }


    public LimelightResults getLatestResults(String limelightName) {

        long start = System.nanoTime();
        LimelightResultsMapper resultsMapper = new LimelightResultsMapper();
        if (mapper == null) {
            mapper = new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
        }

        try {
            resultsMapper = mapper.readValue(getJSONDump(limelightName), LimelightResultsMapper.class);
        } catch (JsonProcessingException e) {
            System.err.println("lljson error: " + e.getMessage());
        }

        long end = System.nanoTime();
        double millis = (end - start) * .000001;
        if (profileJSON) {
            System.out.printf("lljson: %.2f\r\n", millis);
        }

        return resultsMapper.targetingResults;
    }

    public Optional<LimelightRetro> getLatestRetroTarget(String limeLightName) {
        LimelightResults results = getLatestResults(limeLightName);
        if (results != null && results.valid && results.retroReflectiveTargets.length > 0) {
            return Optional.of(results.retroReflectiveTargets[0]);
        }
        return Optional.empty();
    }
    
    public Optional<LimelightFiducial> getLatestFiducialTarget(String limeLightName, String fiducialFamily, double fiducialID) {
        //16H5C - April Tag
        LimelightResults limelightResults = getLatestResults(limeLightName);
        if (limelightResults != null && limelightResults.valid) {
            LimelightFiducial[] fiducialTargets = limelightResults.fiducialTargets;
            if(fiducialTargets.length > 0){
                for (int i = 0; i < limelightResults.fiducialTargets.length; i++) {
                    if (limelightResults.fiducialTargets[i].fiducialFamily == fiducialFamily
                        && limelightResults.fiducialTargets[i].fiducialID == fiducialID) {
                        return Optional.of(limelightResults.fiducialTargets[i]); 
                    }
                }
            }
        }
        return Optional.empty();
    }
    
    public Optional<LimelightDetector> getLatestDetectorTarget(String limeLightName) {
        LimelightResults results = getLatestResults(limeLightName);
        if (results != null && results.valid && results.detectorTargets.length > 0) {
            return Optional.of(results.detectorTargets[0]);
        }
        return Optional.empty();
    }

    public VisionTargetInfo getRobotRelativeTargetInfo(double targetXDegrees, double targetYDegrees) {
        // var rolledAngles = new Translation2d(targetXDegrees, targetYDegrees)
        //     .rotateBy(new Rotation2d(-cameraToRobot.getRotation().getX()));
        // var translation = getTargetTranslation(rolledAngles.getX(), rolledAngles.getY());
        // var distance = translation.getDistance(new Translation2d());
        // var angle = new Rotation2d(translation.getX(), translation.getY());
        // return new VisionTargetInfo(translation, distance, angle);
        return null;
    }

    private Pose3d toPose3D(double[] inData){
        if(inData.length < 6)
        {
            System.err.println("Bad LL 3D Pose Data!");
            return new Pose3d();
        }
        return new Pose3d(
            new Translation3d(inData[0], inData[1], inData[2]),
            new Rotation3d(Units.degreesToRadians(inData[3]), Units.degreesToRadians(inData[4]),
                    Units.degreesToRadians(inData[5])));
    }

    private Pose2d toPose2D(double[] inData){
        if(inData.length < 6)
        {
            System.err.println("Bad LL 2D Pose Data!");
            return new Pose2d();
        }
        Translation2d tran2d = new Translation2d(inData[0], inData[1]);
        Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
        return new Pose2d(tran2d, r2d);
    }

    public URL getLimelightURLString(String tableName, String request) {
        String urlString = "http://" + sanitizeName(tableName) + ".local:5807/" + request;
        URL url;
        try {
            url = new URL(urlString);
            return url;
        } catch (MalformedURLException e) {
            System.err.println("bad LL URL");
        }
        return null;
    }
    /////
    /////

    

    public double getTA(String limelightName) {
        return getLimelightNTDouble(limelightName, "ta");
    }

    public double getLatency_Pipeline(String limelightName) {
        return getLimelightNTDouble(limelightName, "tl");
    }

    public double getLatency_Capture(String limelightName) {
        return getLimelightNTDouble(limelightName, "cl");
    }

    public double getCurrentPipelineIndex(String limelightName) {
        return getLimelightNTDouble(limelightName, "getpipe");
    }

    /**
     * Switch to getBotPose
     * 
     * @param limelightName
     * @return
     */
    @Deprecated
    public double[] getBotpose(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose");
    }

    /**
     * Switch to getBotPose_wpiRed
     * 
     * @param limelightName
     * @return
     */
    @Deprecated
    public double[] getBotpose_wpiRed(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpired");
    }

    /**
     * Switch to getBotPose_wpiBlue
     * 
     * @param limelightName
     * @return
     */
    @Deprecated
    public double[] getBotpose_wpiBlue(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
    }

    public double[] getBotPose(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose");
    }

    public double[] getBotPose_wpiRed(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpired");
    }

    public double[] getBotPose_wpiBlue(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
    }

    public double[] getBotPose_TargetSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_targetspace");
    }

    public double[] getCameraPose_TargetSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "camerapose_targetspace");
    }

    public double[] getTargetPose_CameraSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "targetpose_cameraspace");
    }

    public double[] getTargetPose_RobotSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "targetpose_robotspace");
    }

    public double[] getTargetColor(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "tc");
    }

    public double getFiducialID(String limelightName) {
        return getLimelightNTDouble(limelightName, "tid");
    }

    public double getNeuralClassID(String limelightName) {
        return getLimelightNTDouble(limelightName, "tclass");
    }

    /////
    /////

    public Pose3d getBotPose3d(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose");
        return toPose3D(poseArray);
    }

    public Pose3d getBotPose3d_wpiRed(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose_wpired");
        return toPose3D(poseArray);
    }

    public Pose3d getBotPose3d_wpiBlue(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
        return toPose3D(poseArray);
    }

    public Pose3d getBotPose3d_TargetSpace(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose_targetspace");
        return toPose3D(poseArray);
    }

    public Pose3d getCameraPose3d_TargetSpace(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "camerapose_targetspace");
        return toPose3D(poseArray);
    }

    public Pose3d getTargetPose3d_CameraSpace(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "targetpose_cameraspace");
        return toPose3D(poseArray);
    }

    public Pose3d getTargetPose3d_RobotSpace(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "targetpose_robotspace");
        return toPose3D(poseArray);
    }

    public Pose3d getCameraPose3d_RobotSpace(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "camerapose_robotspace");
        return toPose3D(poseArray);
    }

    /**
     * Gets the Pose2d for easy use with Odometry vision pose estimator
     * (addVisionMeasurement)
     * 
     * @param limelightName
     * @return
     */
    public Pose2d getBotPose2d_wpiBlue(String limelightName) {

        double[] result = getBotPose_wpiBlue(limelightName);
        return toPose2D(result);
    }

    /**
     * Gets the Pose2d for easy use with Odometry vision pose estimator
     * (addVisionMeasurement)
     * 
     * @param limelightName
     * @return
     */
    public Pose2d getBotPose2d_wpiRed(String limelightName) {

        double[] result = getBotPose_wpiRed(limelightName);
        return toPose2D(result);

    }

    /**
     * Gets the Pose2d for easy use with Odometry vision pose estimator
     * (addVisionMeasurement)
     * 
     * @param limelightName
     * @return
     */
    public Pose2d getBotPose2d(String limelightName) {

        double[] result = getBotPose(limelightName);
        return toPose2D(result);

    }

    public boolean getTV(String limelightName) {
        return 1.0 == getLimelightNTDouble(limelightName, "tv");
    }

    /////
    /////

    public void setPipelineIndex(String limelightName, int pipelineIndex) {
        setLimelightNTDouble(limelightName, "pipeline", pipelineIndex);
    }

    /**
     * The LEDs will be controlled by Limelight pipeline settings, and not by robot
     * code.
     */
    public void setLEDMode_PipelineControl(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 0);
    }

    public void setLEDMode_ForceOff(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 1);
    }

    public void setLEDMode_ForceBlink(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 2);
    }

    public void setLEDMode_ForceOn(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 3);
    }

    public void setStreamMode_Standard(String limelightName) {
        setLimelightNTDouble(limelightName, "stream", 0);
    }

    public void setStreamMode_PiPMain(String limelightName) {
        setLimelightNTDouble(limelightName, "stream", 1);
    }

    public void setStreamMode_PiPSecondary(String limelightName) {
        setLimelightNTDouble(limelightName, "stream", 2);
    }

    public void setCameraMode_Processor(String limelightName) {
        setLimelightNTDouble(limelightName, "camMode", 0);
    }
    public void setCameraMode_Driver(String limelightName) {
        setLimelightNTDouble(limelightName, "camMode", 1);
    }


    /**
     * Sets the crop window. The crop window in the UI must be completely open for
     * dynamic cropping to work.
     */
    public void setCropWindow(String limelightName, double cropXMin, double cropXMax, double cropYMin, double cropYMax) {
        double[] entries = new double[4];
        entries[0] = cropXMin;
        entries[1] = cropXMax;
        entries[2] = cropYMin;
        entries[3] = cropYMax;
        setLimelightNTDoubleArray(limelightName, "crop", entries);
    }

    public void setCameraPose_RobotSpace(String limelightName, double forward, double side, double up, double roll, double pitch, double yaw) {
        double[] entries = new double[6];
        entries[0] = forward;
        entries[1] = side;
        entries[2] = up;
        entries[3] = roll;
        entries[4] = pitch;
        entries[5] = yaw;
        setLimelightNTDoubleArray(limelightName, "camerapose_robotspace_set", entries);
    }

    /////
    /////

    public void setPythonScriptData(String limelightName, double[] outgoingPythonData) {
        setLimelightNTDoubleArray(limelightName, "llrobot", outgoingPythonData);
    }

    public double[] getPythonScriptData(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "llpython");
    }

    /////
    /////

    /**
     * Asynchronously take snapshot.
     */
    public CompletableFuture<Boolean> takeSnapshot(String tableName, String snapshotName) {
        return CompletableFuture.supplyAsync(() -> {
            return SYNCH_TAKESNAPSHOT(tableName, snapshotName);
        });
    }

    private boolean SYNCH_TAKESNAPSHOT(String tableName, String snapshotName) {
        URL url = getLimelightURLString(tableName, "capturesnapshot");
        try {
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("GET");
            if (snapshotName != null && snapshotName != "") {
                connection.setRequestProperty("snapname", snapshotName);
            }

            int responseCode = connection.getResponseCode();
            if (responseCode == 200) {
                return true;
            } else {
                System.err.println("Bad LL Request");
            }
        } catch (IOException e) {
            System.err.println(e.getMessage());
        }
        return false;
    }

}