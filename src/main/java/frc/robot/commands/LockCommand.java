package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class LockCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    
    public LockCommand(
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
    drivetrainSubsystem.setXstance();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
