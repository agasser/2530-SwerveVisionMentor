// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ZeroHeadingCommand;
import frc.robot.commands.AprilTagFollowCommand;
import frc.robot.commands.AutonomousCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts.kGrid;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController driverXbox = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem("limelight");
  private final DriveCommand normalDrive = new DriveCommand(drivetrainSubsystem, driverXbox.getHID());
  final ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
  final ShuffleboardLayout visionLayout = visionTab.getLayout("Target", kGrid).withPosition(6, 0).withSize(1, 2);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    drivetrainSubsystem.setDefaultCommand(normalDrive);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driverXbox.y().whileTrue(new AprilTagFollowCommand(drivetrainSubsystem, limelightSubsystem, visionLayout));
    driverXbox.x().onTrue(new ZeroHeadingCommand(drivetrainSubsystem));
    driverXbox.a().onTrue(Commands.runOnce(() -> drivetrainSubsystem.resetOdometry(new Pose2d()), drivetrainSubsystem));
    driverXbox.b().whileTrue(Commands.run(drivetrainSubsystem::setXstance, drivetrainSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    public Command getAutonomousCommand() {
      return new AutonomousCommand(drivetrainSubsystem);
    }

  }
