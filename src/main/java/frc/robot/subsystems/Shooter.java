package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase{
    private CANSparkMax ShooterMotorR = new CANSparkMax(Constants.ShooterConstants.kRightShooterMotorPort, MotorType.kBrushless);
    private CANSparkMax ShooterMotorL = new CANSparkMax(Constants.ShooterConstants.kLeftShooterMotorPort, MotorType.kBrushless);



    public Shooter(){   
        ShooterMotorL.restoreFactoryDefaults();
        ShooterMotorR.restoreFactoryDefaults();

    }

    public void stopMotor(){    
        ShooterMotorL.stopMotor();
        ShooterMotorR.stopMotor();
    }


    public void setShooterR(double speed){
        ShooterMotorR.set(speed);
    }

    public void setShooterL(double speed){
        ShooterMotorL.set(speed);
    }

    @Override
    public void periodic(){
    
    }


}
