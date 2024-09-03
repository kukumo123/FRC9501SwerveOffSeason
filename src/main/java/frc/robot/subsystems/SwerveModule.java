package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    private SwerveModuleState currentState = new SwerveModuleState();
    private final TalonFX driveMotor;
    private final CANSparkMax turningMotor;
    private SwerveModulePosition currentPosition = new SwerveModulePosition();
    private final boolean absoluteEncoderReversed;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final CANcoder absoluteEncoder;

    private final double absoluteEncoderOffsetRad;


    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        absoluteEncoder = new CANcoder(absoluteEncoderId);

        driveMotor = new TalonFX(driveMotorId);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        turningEncoder = turningMotor.getEncoder();

        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        turningMotor.setIdleMode(IdleMode.kBrake);
        turningMotor.burnFlash();
        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getPosition().getValue() * Constants.ModuleConstants.kDriveMotorGearRatio;
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValue() * Constants.ModuleConstants.kDriveEncoderRPM2MeterPerSec;
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

      public void setTargetState(SwerveModuleState targetState) {
        // Optimize the state
        currentState = SwerveModuleState.optimize(targetState, currentState.angle);
  
        currentPosition = new SwerveModulePosition(currentPosition.distanceMeters + (currentState.speedMetersPerSecond * 0.02), currentState.angle);
      }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition().getValue();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return currentState;
    }
      /** Sets whether brake mode is enabled. */
    public void setBrakeMode(boolean enabled) {
        IdleMode idleMode = enabled?IdleMode.kBrake:IdleMode.kCoast;
        
        turningMotor.setIdleMode(idleMode);
    }

    public SwerveModuleState setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return new SwerveModuleState();
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        double value = turningPidController.calculate(getTurningPosition(), state.angle.getRadians());
        turningMotor.set(value);
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
        return state;
    }

    public SwerveModulePosition getPosition() {
        return currentPosition;
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
