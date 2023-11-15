package frc.robot.commands;

import static edu.wpi.first.math.MathUtil.applyDeadband;
import static edu.wpi.first.math.MathUtil.clamp;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final XboxController xbox;

    private SlewRateLimiter dsratelimiter = new SlewRateLimiter(4);

    private double DRIVE_MULT = 1.0;
    private final double SLOWMODE_MULT = 0.25;

    public DriveCommand(DrivetrainSubsystem drivetrainSubsystem, XboxController xbox) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.xbox = xbox;

        dsratelimiter.reset(SLOWMODE_MULT);

        addRequirements(drivetrainSubsystem);
    }

    public Translation2d DeadBand(Translation2d input, double deadzone) {
        double mag = input.getNorm();
        Translation2d norm = input.div(mag);

        if (mag < deadzone) {
            return new Translation2d(0.0, 0.0);
        } else {
            // TODO: Check is it sqrt2 or 1.0...
            Translation2d result = norm.times((mag - deadzone) / (1.0 - deadzone));
            return new Translation2d(
                    clamp(result.getX(), -1.0, 1.0),
                    clamp(result.getY(), -1.0, 1.0));
        }
    }

    @Override
    public void execute() {
        Translation2d xyRaw = new Translation2d(-xbox.getLeftY(), -xbox.getLeftX());
        Translation2d xySpeed = DeadBand(xyRaw, 0.15);
        double zSpeed = applyDeadband(-xbox.getRightX(), 0.1);
        double xSpeed = xySpeed.getX(); // xbox.getLeftX();
        double ySpeed = xySpeed.getY(); // xbox.getLeftY();

        SmartDashboard.putNumber("xySpeed norm", xySpeed.getNorm());

        // double mag_xy = Math.sqrt(xSpeed*xSpeed + ySpeed*ySpeed);

        // xSpeed = mag_xy > 0.15 ? xSpeed : 0.0;
        // ySpeed = mag_xy > 0.15 ? ySpeed : 0.0;
        // zSpeed = Math.abs(zSpeed) > 0.15 ? zSpeed : 0.0;

        // TODO: Full speed!
        xSpeed *= DrivetrainConstants.XY_SPEED_LIMIT * DrivetrainConstants.MAX_ROBOT_VELOCITY;
        ySpeed *= DrivetrainConstants.XY_SPEED_LIMIT * DrivetrainConstants.MAX_ROBOT_VELOCITY;
        zSpeed *= DrivetrainConstants.Z_SPEED_LIMIT * DrivetrainConstants.MAX_ROBOT_RAD_VELOCITY;

        // double dmult = dsratelimiter.calculate(xbox.getRightBumper() ? 1.0 :
        // SLOWMODE_MULT);
        double dmult = dsratelimiter
                .calculate((DRIVE_MULT - SLOWMODE_MULT) * xbox.getRightTriggerAxis() + SLOWMODE_MULT);
        xSpeed *= dmult;
        ySpeed *= dmult;
        zSpeed *= dmult;

        ChassisSpeeds speeds;

        // Drive Non Field Oriented
        if (!xbox.getLeftBumper()) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, drivetrainSubsystem.getRotation2d());
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, zSpeed);
        }

        if((xyRaw.getNorm() > 0.15)){
            SwerveModuleState[] calculatedModuleStates = DrivetrainConstants.KINEMATICS.toSwerveModuleStates(speeds);
            drivetrainSubsystem.setModules(calculatedModuleStates);
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stopDrive();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

