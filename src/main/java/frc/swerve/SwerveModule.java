package frc.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DrivetrainConstants;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;

    private final RelativeEncoder driveMotorEncoder;
    private final RelativeEncoder steerMotorEncoder;

    private final CANcoder absoluteEncoder;

    private final double motorOffsetRadians;
    private final boolean isAbsoluteEncoderReversed;
    private final boolean motor_inv;

    private final PIDController steerPID;

    private static int moduleNumber = 0;
    int thisModuleNumber;

    SlewRateLimiter turnratelimiter = new SlewRateLimiter(4.d);

    public SwerveModule(int steerCanID, int driveCanID, int absoluteEncoderPort, double motorOffsetRadians,
            boolean isAbsoluteEncoderReversed, boolean motorReversed) {
        driveMotor = new CANSparkMax(driveCanID, MotorType.kBrushless);
        driveMotor.setInverted(false);
        driveMotor.setIdleMode(IdleMode.kBrake);
        steerMotor = new CANSparkMax(steerCanID, MotorType.kBrushless);
        steerMotor.setIdleMode(IdleMode.kBrake);
        steerMotor.setInverted(false);

        this.motor_inv = motorReversed;
        driveMotorEncoder = driveMotor.getEncoder();
        steerMotorEncoder = steerMotor.getEncoder();

        absoluteEncoder = new CANcoder(absoluteEncoderPort);

        this.motorOffsetRadians = motorOffsetRadians;
        this.isAbsoluteEncoderReversed = isAbsoluteEncoderReversed;

        driveMotorEncoder.setPositionConversionFactor(DrivetrainConstants.DRIVE_ROTATION_TO_METER);
        driveMotorEncoder.setVelocityConversionFactor(DrivetrainConstants.DRIVE_METERS_PER_MINUTE);

        steerMotorEncoder.setPositionConversionFactor(DrivetrainConstants.STEER_ROTATION_TO_RADIANS);
        steerMotorEncoder.setVelocityConversionFactor(DrivetrainConstants.STEER_RADIANS_PER_MINUTE);

        steerPID = new PIDController(DrivetrainConstants.MODULE_KP, 0, DrivetrainConstants.MODULE_KD);
        steerPID.enableContinuousInput(-Math.PI, Math.PI);

        thisModuleNumber = moduleNumber;
        moduleNumber++;

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotorEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveMotorEncoder.getVelocity();
    }

    public double getSteerPosition() {
        return steerMotorEncoder.getPosition();
    }

    public double getSteerVelocity() {
        return steerMotorEncoder.getVelocity();
    }

    public double getAbsoluteEncoderPosition() {
        double angle = absoluteEncoder.getAbsolutePosition().getValue() * (Math.PI / 180.d);
        angle -= motorOffsetRadians;
        return angle * (isAbsoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveMotorEncoder.setPosition(0);
        steerMotorEncoder.setPosition(getAbsoluteEncoderPosition());
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerPosition()));
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getSteerPosition()));
    }

    public void setModuleStateRaw(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, new Rotation2d(getSteerPosition()));
        double drive_command = state.speedMetersPerSecond / DrivetrainConstants.MAX_MODULE_VELOCITY;
        driveMotor.set(drive_command * (motor_inv ? -1.0 : 1.0));

        steerMotor.set(steerPID.calculate(getSteerPosition(), state.angle.getRadians()));

        SmartDashboard.putNumber("Drive" + thisModuleNumber, drive_command);
    }

    public void setModuleState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        setModuleStateRaw(state);
    }

    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
    }
}
