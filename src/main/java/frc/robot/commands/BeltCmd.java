package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Belt;

public class BeltCmd extends Command{
    private Belt belt;
    private double beltSpeed;

    public BeltCmd(Belt belt, double beltSpeed){
        this.belt = belt;
        this.beltSpeed = beltSpeed;
        addRequirements(belt);
    }
    
    @Override
    public void initialize(){
    }

    //循環(進到Command後循環)
    @Override
    public void execute(){
        belt.setBeltMotorSpeed(this.beltSpeed);
    }

    @Override
    public void end(boolean interrupted){
        belt.stopMotor();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
