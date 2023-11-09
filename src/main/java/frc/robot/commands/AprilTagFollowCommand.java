package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.limelight.LimelightFiducial;
import frc.robot.limelight.LimelightHelperFunctions;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AprilTagFollowCommand extends CommandBase {

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(8, 8);

    private static final int TAG_TO_CHASE = 1;
    private final LimelightHelperFunctions limelightHelperFunctions;
    private static final Transform3d TAG_TO_GOAL = 
      new Transform3d(
          new Translation3d(1.5, 0.0, 0.0),
          new Rotation3d(0.0, 0.0, Math.PI));
    
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final LimelightSubsystem limelightSubsystem;
    private LimelightFiducial lastTarget;


    private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

    public AprilTagFollowCommand(
        DrivetrainSubsystem drivetrainSubsystem,
        LimelightSubsystem limelightSubsystems) 
    {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.limelightSubsystem = limelightSubsystems;
        limelightHelperFunctions = new LimelightHelperFunctions();

        addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    lastTarget = null;
  }

  //Need help here does not know how to move robot using limelight co-ordinates
  @Override
  public void execute() {
    // If the target is visible, get the new translation. If the target isn't visible we'll use the last known translation.
    var firstTarget = LimelightHelperFunctions.getLatestFiducialTarget(LimelightConstants.limeLightName, "16H5C", TAG_TO_CHASE);
    if(firstTarget.isPresent()){
         // This is new target data, so recalculate the goal
         lastTarget = firstTarget.get();

         var robotPose = LimelightHelperFunctions.getBotPose3d(LimelightConstants.limeLightName);
            
         // Transform the robot's pose to find the camera's pose
         var cameraPose = LimelightHelperFunctions.getTargetPose3d_CameraSpace(LimelightConstants.limeLightName);
 
         // Trasnform the camera's pose to the target's pose
         var targetPose = LimelightHelperFunctions.getBotPose3d_TargetSpace(LimelightConstants.limeLightName);
         
         // Transform the tag's pose to set our goal
         var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();
 
         // Drive
         xController.setGoal(goalPose.getX());
         yController.setGoal(goalPose.getY());
         omegaController.setGoal(goalPose.getRotation().getRadians());
         //swerveSubsystem.drive(lastTarget.targetXDegrees, lastTarget.targetYDegrees, omegaSpeed, true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stopDrive();
  }
    
}