 package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCmd extends Command{
    private Intake intake;
    private double intakeSpeed;

    public IntakeCmd(Intake intake, double intakeSpeed){
        this.intake = intake;
        this.intakeSpeed = intakeSpeed;

        addRequirements(intake);
    }

    //初始化(進到Command會跑一次)
    @Override
    public void initialize(){
        System.out.println("IntakeCmd Started!");   
    }

    //循環(進到Command後循環)
    @Override
    public void execute(){
        intake.setIntakeMotor(this.intakeSpeed);
    }

    @Override
    public void end(boolean interrupted){
        intake.stopMotor();
        System.out.println("IntakeCmd finished!");
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}