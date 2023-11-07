package frc.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DrivetrainConstants;

public class SwerveModule {
    private static int moduleNumber = 0;
    
    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;

    private final RelativeEncoder driveMotorEncoder;
    private final RelativeEncoder steerMotorEncoder;

    private final CANcoder absoluteEncoder;

    private final SparkMaxPIDController steerPID;

    private int thisModuleNumber;

    public SwerveModule(int steerCanID, int driveCanID, int absoluteEncoderPort, double motorOffsetRadians,
            boolean isAbsoluteEncoderReversed, boolean motorReversed) {
        driveMotor = new CANSparkMax(driveCanID, MotorType.kBrushless);
        driveMotor.setInverted(false);
        driveMotor.setIdleMode(IdleMode.kBrake);
        steerMotor = new CANSparkMax(steerCanID, MotorType.kBrushless);
        steerMotor.setIdleMode(IdleMode.kBrake);
        steerMotor.setInverted(false);

        driveMotorEncoder = driveMotor.getEncoder();
        steerMotorEncoder = steerMotor.getEncoder();

        absoluteEncoder = new CANcoder(absoluteEncoderPort);
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        // Set the CANCoder magnetic offset. This is the inverse of the ROTATIONS the sensor reads when the wheel is pointed straight forward.
        canCoderConfig.MagnetSensor.MagnetOffset = Units.radiansToRotations(motorOffsetRadians);
        // Set CANCoder to return direction from [-.5, .5) - straight forward should be 0
        canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        // Set the CANCoder phase / direction
        canCoderConfig.MagnetSensor.SensorDirection =
            isAbsoluteEncoderReversed ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;

        absoluteEncoder.getConfigurator().apply(canCoderConfig);

        // Drive distance in meters
        driveMotorEncoder.setPositionConversionFactor(DrivetrainConstants.DRIVE_ROTATION_TO_METER);
        // Drive velocity in meters per second
        driveMotorEncoder.setVelocityConversionFactor(DrivetrainConstants.DRIVE_METERS_PER_SECOND);

        // Steer position in rotations
        steerMotorEncoder.setPositionConversionFactor(DrivetrainConstants.STEERING_GEAR_RATIO);

        steerPID = steerMotor.getPIDController();
        steerPID.setP(DrivetrainConstants.MODULE_KP);
        steerPID.setD(DrivetrainConstants.MODULE_KD);
        steerPID.setPositionPIDWrappingEnabled(true);
        steerPID.setPositionPIDWrappingMaxInput(0.5);
        steerPID.setPositionPIDWrappingMinInput(-0.5);

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

    /**
     * Gets the absolute encoder (CANCoder) position
     * @return position in rotations [0, 1)
     */
    private double getAbsoluteEncoderPosition() {
        return absoluteEncoder.getAbsolutePosition().waitForUpdate(0.2).getValue();
    }

    private void resetEncoders() {
        driveMotorEncoder.setPosition(0);
        steerMotorEncoder.setPosition(Units.rotationsToRadians(getAbsoluteEncoderPosition()));
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
        driveMotor.set(drive_command);
        steerPID.setReference(state.angle.getRotations(), ControlType.kPosition);

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
