// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANcoder;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

/**
 * Basic simulation of a swerve subsystem with the methods needed by PathPlanner
 */
public class SwerveSubsystem extends SubsystemBase{
  private final SwerveModule[] modules = new SwerveModule[4];
  public SwerveDriveKinematics kinematics;
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveDrivePoseEstimator m_poseEstimator;
  private SwerveDriveOdometry odometry;  
  private Field2d field = new Field2d();
  private AHRS gyro;
  private SwerveModulePosition[] lastModulePositions;// For delta tracking
  private SwerveModulePosition[] modulePositions;
  private SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    private final CANcoder FLcancoder = new CANcoder(9, "rio");
    private final CANcoder BLcancoder = new CANcoder(10, "rio");
    private final CANcoder FRcancoder = new CANcoder(11, "rio");
    private final CANcoder BRcancoder = new CANcoder(12, "rio");
    private Rotation2d currentRotation = new Rotation2d();


  public SwerveSubsystem() {
    gyro = new AHRS(SPI.Port.kMXP);
    new Thread(() -> {
      try {
          Thread.sleep(1000);
          zeroHeading();
      } catch (Exception e) {
      }
  }).start();

    setupPathPlanner();
    
        modules[0] = new SwerveModule(
                DriveConstants.kFrontLeftDriveMotorPort,
                DriveConstants.kFrontLeftTurningMotorPort,
                DriveConstants.kFrontLeftDriveEncoderReversed,
                DriveConstants.kFrontLeftTurningEncoderReversed,
                DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
                DriveConstants.kFrontLeftAbsoluteEncoderOffsetRad,
                DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

        modules[1] = new SwerveModule(
                DriveConstants.kFrontRightDriveMotorPort,
                DriveConstants.kFrontRightTurningMotorPort,
                DriveConstants.kFrontRightDriveEncoderReversed,
                DriveConstants.kFrontRightTurningEncoderReversed,
                DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
                DriveConstants.kFrontRightAbsoluteEncoderOffsetRad,
                DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

        modules[2] = new SwerveModule(
                DriveConstants.kBackLeftDriveMotorPort,
                DriveConstants.kBackLeftTurningMotorPort,
                DriveConstants.kBackLeftDriveEncoderReversed,
                DriveConstants.kBackLeftTurningEncoderReversed,
                DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
                DriveConstants.kBackLeftAbsoluteEncoderOffsetRad,
                DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

        modules[3] = new SwerveModule(
                DriveConstants.kBackRightDriveMotorPort,
                DriveConstants.kBackRightTurningMotorPort,
                DriveConstants.kBackRightDriveEncoderReversed,
                DriveConstants.kBackRightTurningEncoderReversed,
                DriveConstants.kBackRightDriveAbsoluteEncoderPort,
                DriveConstants.kBackRightAbsoluteEncoderOffsetRad,
                DriveConstants.kBackRightDriveAbsoluteEncoderReversed);
                lastModulePositions = // For delta tracking
                new SwerveModulePosition[] {
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition()
                };
    kinematics = new SwerveDriveKinematics(
      Constants.ModuleConstants.flModuleOffset, 
      Constants.ModuleConstants.frModuleOffset, 
      Constants.ModuleConstants.blModuleOffset, 
      Constants.ModuleConstants.brModuleOffset
    );
    odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getPositions());
    modulePositions = getModulePositions();
    m_poseEstimator = new SwerveDrivePoseEstimator(
      kinematics, 
      rawGyroRotation, lastModulePositions, new Pose2d());   
    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    SmartDashboard.putData("Field", field);
  }

  public void zeroHeading() {
    if (gyro.isConnected()) {
        gyro.reset();
    }
}

  @Override
  public void periodic() {

    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      modulePositions[moduleIndex] = modules[moduleIndex].getPosition();
      moduleDeltas[moduleIndex] = new SwerveModulePosition(
              modulePositions[moduleIndex].distanceMeters
                      - lastModulePositions[moduleIndex].distanceMeters,
              modulePositions[moduleIndex].angle);
      lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      SmartDashboard.putString("AbsoluteEncoder[" + moduleIndex + "]",
              String.valueOf(modules[moduleIndex].getAbsoluteEncoderRad()));
      SmartDashboard.putString("RelativeEncoder[" + moduleIndex + "]",
              String.valueOf(modules[moduleIndex].getTurningPosition()));


    odometry.update(gyro.getRotation2d(), getPositions());

    field.setRobotPose(getPose());
    }
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), getPositions(), pose);
  }

  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
    setStates(targetStates);
  }

      public void getTurningEncoderPosition(){
        SmartDashboard.putNumber("FL", FLcancoder.getAbsolutePosition().getValue()*(Math.PI*2));
        SmartDashboard.putNumber("BL", BLcancoder.getAbsolutePosition().getValue()*(Math.PI*2));
        SmartDashboard.putNumber("FR", FRcancoder.getAbsolutePosition().getValue()*(Math.PI*2));
        SmartDashboard.putNumber("BR", BRcancoder.getAbsolutePosition().getValue()*(Math.PI*2));
    }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
        states[i] = modules[i].getPosition();
    }
    return states;
}

  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.ModuleConstants.maxModuleSpeed);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setTargetState(targetStates[i]);
    }
  }

  public Rotation2d getKinematicsRotation2d() {
    return currentRotation;
  
  }

  public void updateRotation(double angularVelRps){
    currentRotation = currentRotation.plus(new Rotation2d(angularVelRps * 0.02));
  }


public Rotation2d getRotation2d() {
  return Rotation2d.fromDegrees(getHeading());
}

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360);
}

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  public void runVelocity(ChassisSpeeds speeds) {
    // 將連續時間的底盤速度轉換為離散時間速度
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = setModuleStates(setpointStates);

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized",
            optimizedSetpointStates);
}

public SwerveModuleState[] setModuleStates(SwerveModuleState[] desiredStates) {
  // 重新規範（標準化）萬向輪驅動模組速度
  SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

  // 設定馬達速度
  SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
  for (int i = 0; i < 4; i++) {
      optimizedSetpointStates[i] = modules[i].setDesiredState(desiredStates[i]);
  }
  return optimizedSetpointStates;
}


  public void stopModules() {
    for (int i = 0; i < 4; i++) {
        modules[i].stop();
    }
}

public void resetEncoder(){
  modules[0].resetEncoders();
  modules[1].resetEncoders();
  modules[2].resetEncoders();
  modules[3].resetEncoders();
}
  public SwerveModulePosition[] getModulePosition(){
    return new SwerveModulePosition[]{
        modules[0].getPosition(),
        modules[1].getPosition(),
        modules[2].getPosition(),
        modules[3].getPosition()
    };
}
  public void resetOdomtry(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), getModulePosition(), pose);
}

      public Command getAutonomousCommand(String pathName, boolean setOdomToStart)
    {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      if (setOdomToStart)
      {
        resetOdomtry(new Pose2d());
      }
      return AutoBuilder.followPath(path);
    }

      public void setupPathPlanner()
    {
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetPose,
            this::getRobotVelocity,     
            this::driveRobotRelative,
            new HolonomicPathFollowerConfig( 
                                            new PIDConstants( 0.01, 0, 0, 0),
                                            new PIDConstants(0.03, 0, 0, 0),
                                            Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond,
                                            0.2,
                                            new ReplanningConfig()
            ),
            () -> {
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Blue : true;
            },
            this
        );
    }

    public ChassisSpeeds getRobotVelocity() {
      return kinematics.toChassisSpeeds(this.getSweveModuleStates());
  }

  public SwerveModuleState[] getSweveModuleStates(){
    return new SwerveModuleState[] {
    modules[0].getState(),
    modules[1].getState(),
    modules[2].getState(),
    modules[3].getState()
    };
}
}

