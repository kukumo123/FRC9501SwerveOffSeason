package frc.robot;


import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.BeltCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.ShooterCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Elvator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;


public class RobotContainer {


  
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    private final POVButton pov0 = new POVButton(driverJoytick, 0);
    private final POVButton pov1 = new POVButton(driverJoytick, 270);


    
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Elvator elvator = new Elvator();
    private final Belt belt = new Belt();
    private static final String kDefaultAuto = "Default";
    private static final String middleStart = "Auto";
    private static final String RightStart = "Auto2";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    public RobotContainer() {
        
        NamedCommands.registerCommand("Intake", new IntakeCmd(intake, -1).withTimeout(2));
        NamedCommands.registerCommand("Cum", new SequentialCommandGroup(
          new ShooterCmd(shooter, 1, -1).withTimeout(0.5),
          new ParallelCommandGroup(new BeltCmd(belt, 1), new ShooterCmd(shooter, 1, -1)).withTimeout(1.3) 
        ));
        NamedCommands.registerCommand("belt", new BeltCmd(belt, 1).withTimeout(2));
        NamedCommands.registerCommand("shooter", new ShooterCmd(shooter, 1, -1).withTimeout(15));

        
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("Auto", middleStart);
        m_chooser.addOption("Auto2", RightStart);
        SmartDashboard.putData("Auto choices", m_chooser);  
        m_autoSelected =  m_chooser.getSelected();
        System.out.println("Auto Selected" + m_autoSelected);

        switch (m_autoSelected) {
          case middleStart:
            break;
          case RightStart:
            break;
          case kDefaultAuto:
          default:
          break;
        }

        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
        configureButtonBindings();     
    }

  
    private void configureButtonBindings() {

            // The rotation component in these poses represents the direction of travel
      // Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      // Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());
    
    // new JoystickButton(driverJoytick, Button.kX.value).whileTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading(),swerveSubsystem));
    pov1.whileTrue(new ShooterCmd(shooter, 0.25, -0.25));
    pov1.whileTrue(new BeltCmd(belt, 0.35));
    new JoystickButton(driverJoytick, Button.kStart.value).whileTrue(new IntakeCmd(intake, -1));
    new JoystickButton(driverJoytick, Button.kBack.value).whileTrue(new IntakeCmd(intake, 1));
    new JoystickButton(driverJoytick, Button.kRightBumper.value).whileTrue(new  ShooterCmd(shooter, -1, 1));
    new JoystickButton(driverJoytick, Button.kA.value).onTrue(new InstantCommand(() -> elvator.LetRobotUP()));
    new JoystickButton(driverJoytick, Button.kB.value).onTrue(new InstantCommand(() -> elvator.LetRobotDOWN()));
    new JoystickButton(driverJoytick, Button.kLeftBumper.value).whileTrue(new BeltCmd(belt, 0.9));
    pov0.whileTrue(new BeltCmd(belt, -0.6));
    }


      public PathPlannerAuto getAutonomousCommand(){
        return new PathPlannerAuto(m_chooser.getSelected());
      }
  }