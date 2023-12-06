package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class ZeroHeadingCommand extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;

    public ZeroHeadingCommand(
        SwerveSubsystem swerveSubsystem) 
    {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    
  @Override
  public void initialize() {
    super.initialize();
  }

  @Override
  public void execute() {
    swerveSubsystem.zeroHeading();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
    
}
