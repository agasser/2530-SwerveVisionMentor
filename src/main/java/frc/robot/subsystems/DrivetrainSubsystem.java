package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.swerve.SwerveModule;

public class DrivetrainSubsystem extends SubsystemBase {
    SwerveModule frontLeft = new SwerveModule(DrivetrainConstants.FL_STEER_ID, DrivetrainConstants.FL_DRIVE_ID,
            DrivetrainConstants.FL_ABSOLUTE_ENCODER_PORT, DrivetrainConstants.FL_OFFSET_RADIANS,
            DrivetrainConstants.FL_ABSOLUTE_ENCODER_REVERSED,
            DrivetrainConstants.FL_MOTOR_REVERSED);

    SwerveModule frontRight = new SwerveModule(DrivetrainConstants.FR_STEER_ID, DrivetrainConstants.FR_DRIVE_ID,
            DrivetrainConstants.FR_ABSOLUTE_ENCODER_PORT, DrivetrainConstants.FR_OFFSET_RADIANS,
            DrivetrainConstants.FR_ABSOLUTE_ENCODER_REVERSED,
            DrivetrainConstants.FR_MOTOR_REVERSED);

    SwerveModule backRight = new SwerveModule(DrivetrainConstants.BR_STEER_ID, DrivetrainConstants.BR_DRIVE_ID,
            DrivetrainConstants.BR_ABSOLUTE_ENCODER_PORT, DrivetrainConstants.BR_OFFSET_RADIANS,
            DrivetrainConstants.BR_ABSOLUTE_ENCODER_REVERSED,
            DrivetrainConstants.BR_MOTOR_REVERSED);

    SwerveModule backLeft = new SwerveModule(DrivetrainConstants.BL_STEER_ID, DrivetrainConstants.BL_DRIVE_ID,
            DrivetrainConstants.BL_ABSOLUTE_ENCODER_PORT, DrivetrainConstants.BL_OFFSET_RADIANS,
            DrivetrainConstants.BL_ABSOLUTE_ENCODER_REVERSED,
            DrivetrainConstants.BL_MOTOR_REVERSED);

    private final AHRS navX = new AHRS(SPI.Port.kMXP);

    private Field2d field = new Field2d();

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(DrivetrainConstants.KINEMATICS, getRotation2d(),
            getModulePositions());

    public DrivetrainSubsystem() {
        zeroHeading();
        // new Thread(() -> {
        // try {
        // Thread.sleep(1000);
        // zeroHeading();
        // } catch (Exception e) {
        // // TODO: handle exception
        // }
        // }).start();
    }

    @Override
    public void periodic() {
        double rads = getPose().getRotation().getRadians(); 
        odometry.update(geRotation2dOdometry(), getModulePositions());
        field.setRobotPose(getPose());
        SmartDashboard.putData("Field", field);

        SmartDashboard.putString("Robot Pose",
                getPose().toString());

        SmartDashboard.putNumber("Spin Velocity", (getPose().getRotation().getRadians() - rads) / 0.02);
    }

    public void zeroHeading() {
        navX.reset();
    }

    public void setHeading(double deg) {
        zeroHeading();
        navX.setAngleAdjustment(deg);
    }

    public Pose2d getPose() {
        Pose2d p = odometry.getPoseMeters();
        // - Y!!!
        p = new Pose2d(p.getX(), p.getY(), p.getRotation().rotateBy(new Rotation2d(Math.PI / 2.0)));
        return p;
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public double getHeading() {
        return Units.degreesToRadians(Math.IEEEremainder(navX.getAngle(), 360));
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(getHeading());
    }

    public Rotation2d geRotation2dOdometry() {
        return new Rotation2d(getHeading() + Math.PI / 2);
    }

    public void stopDrive() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModules(SwerveModuleState[] states) {
        // Normalize speeds so they are all obtainable
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.MAX_MODULE_VELOCITY);
        frontLeft.setModuleState(states[3]);
        frontRight.setModuleState(states[1]);
        backLeft.setModuleState(states[2]);
        backRight.setModuleState(states[0]);
    }

    public void setChassisSpeedsAUTO(ChassisSpeeds speeds) {
        double tmp = -speeds.vxMetersPerSecond;
        speeds.vxMetersPerSecond = -speeds.vyMetersPerSecond;
        speeds.vyMetersPerSecond = tmp; // FORWARDS
        // SmartDashboard.putNumber("Radians Chassis CMD",
        // speeds.omegaRadiansPerSecond);
        SwerveModuleState[] states = DrivetrainConstants.KINEMATICS.toSwerveModuleStates(speeds);
        setModules(states);
    }

    public void setXstance() {
        frontLeft.setModuleStateRaw(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setModuleStateRaw(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backLeft.setModuleStateRaw(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRight.setModuleStateRaw(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public ChassisSpeeds getChassisSpeeds() {
        ChassisSpeeds speeds = DrivetrainConstants.KINEMATICS.toChassisSpeeds(frontLeft.getModuleState(),
                frontRight.getModuleState(),
                backLeft.getModuleState(), backRight.getModuleState());

        return speeds;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        states[3] = frontLeft.getModulePosition();
        states[1] = frontRight.getModulePosition();
        states[2] = backLeft.getModulePosition();
        states[0] = backRight.getModulePosition();

        return states;
    }
}
