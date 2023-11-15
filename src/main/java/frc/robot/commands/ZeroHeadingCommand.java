package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ZeroHeadingCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;

    public ZeroHeadingCommand(
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
    drivetrainSubsystem.zeroHeading();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
    
}
