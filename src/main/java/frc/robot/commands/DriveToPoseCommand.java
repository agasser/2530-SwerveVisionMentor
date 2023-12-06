package frc.robot.commands;

import static frc.robot.Constants.AutoConstants.THETA_kD;
import static frc.robot.Constants.AutoConstants.THETA_kI;
import static frc.robot.Constants.AutoConstants.THETA_kP;
import static frc.robot.Constants.AutoConstants.X_kD;
import static frc.robot.Constants.AutoConstants.X_kI;
import static frc.robot.Constants.AutoConstants.X_kP;
import static frc.robot.Constants.AutoConstants.Y_kD;
import static frc.robot.Constants.AutoConstants.Y_kI;
import static frc.robot.Constants.AutoConstants.Y_kP;
import static frc.robot.Constants.VisionConstants.FIELD_WIDTH_METERS;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Command to drive to a pose.
 */
public class DriveToPoseCommand extends CommandBase {
  
  private static final double TRANSLATION_TOLERANCE = 0.02;
  private static final double THETA_TOLERANCE = Units.degreesToRadians(2.0);

  /** Default constraints are 90% of max speed, accelerate to full speed in 1/3 second */
  private static final TrapezoidProfile.Constraints DEFAULT_XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
      DriveConstants.MAX_ROBOT_VELOCITY * 0.5,
      DriveConstants.MAX_ROBOT_VELOCITY);
  private static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
      DriveConstants.MAX_ROBOT_RAD_VELOCITY * 0.4,
      DriveConstants.MAX_ROBOT_RAD_VELOCITY);

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Pose2d> poseProvider;
  private final Pose2d goalPose;
  private final boolean useAllianceColor;

  public DriveToPoseCommand(
    SwerveSubsystem swerveSubsystem,
    Supplier<Pose2d> poseProvider,
    Pose2d goalPose,
    boolean useAllianceColor) {
    this(swerveSubsystem, poseProvider,goalPose, DEFAULT_XY_CONSTRAINTS, DEFAULT_OMEGA_CONSTRAINTS, useAllianceColor);
  }

  public DriveToPoseCommand(
        SwerveSubsystem swerveSubsystem,
        Supplier<Pose2d> poseProvider,
        Pose2d goalPose,
        TrapezoidProfile.Constraints xyConstraints,
        TrapezoidProfile.Constraints omegaConstraints,
        boolean useAllianceColor) {
    this.swerveSubsystem = swerveSubsystem;
    this.poseProvider = poseProvider;
    this.goalPose = goalPose;
    this.useAllianceColor = useAllianceColor;

    xController = new ProfiledPIDController(X_kP, X_kI, X_kD, xyConstraints);
    yController = new ProfiledPIDController(Y_kP, Y_kI, Y_kD, xyConstraints);
    xController.setTolerance(TRANSLATION_TOLERANCE);
    yController.setTolerance(TRANSLATION_TOLERANCE);
    thetaController = new ProfiledPIDController(THETA_kP, THETA_kI, THETA_kD, omegaConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(THETA_TOLERANCE);

    addRequirements(swerveSubsystem);
  }


  @Override
  public void initialize() {
    resetPIDControllers();
    var pose = goalPose;
    if (useAllianceColor && DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      Translation2d transformedTranslation = new Translation2d(pose.getX(), FIELD_WIDTH_METERS - pose.getY());
      Rotation2d transformedHeading = pose.getRotation().times(-1);
      pose = new Pose2d(transformedTranslation, transformedHeading);
    }
    thetaController.setGoal(pose.getRotation().getRadians());
    xController.setGoal(pose.getX());
    yController.setGoal(pose.getY());  }

  public boolean atGoal() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }

  private void resetPIDControllers() {
    var robotPose = poseProvider.get();
    thetaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  @Override
  public void execute() {
    ChassisSpeeds speeds;
    var robotPose = poseProvider.get();
    // Drive to the goal
    var xSpeed = xController.calculate(robotPose.getX());
    if (xController.atGoal()) {
      xSpeed = 0;
    }

    var ySpeed = yController.calculate(robotPose.getY());
    if (yController.atGoal()) {
      ySpeed = 0;
    }

    var omegaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
    if (thetaController.atGoal()) {
      omegaSpeed = 0;
    }

    // Drive Non Field Oriented
    //if (!xbox.getLeftBumper()) {
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, swerveSubsystem.getRotation2d());
    //} else {
    //    speeds = new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed);
    //}

    SwerveModuleState[] calculatedModuleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
    swerveSubsystem.setModules(calculatedModuleStates);
  }

  @Override
  public boolean isFinished() {
    return atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopDrive();
  }

}