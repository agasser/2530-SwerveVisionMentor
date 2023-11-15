package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ResetOdometryCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    
    public ResetOdometryCommand(
        DrivetrainSubsystem drivetrainSubsystem) 
    {
        this.drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
    }

    
  @Override
  public void initialize() {
    super.initialize();
  }

  @Override
  public void execute() {
    drivetrainSubsystem.resetOdometry(new Pose2d());
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
