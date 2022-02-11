package frc.robot.subsystems.Climber;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;

public class Climber{
    public CANSparkMax climberMotor;

    public Climber(){
        climberMotor = new CANSparkMax(Constants.Climber.kClimberPort, CANSparkMax.MotorType.kBrushless);

    }

    public void set(double power){
        climberMotor.set(power);
        
    }
}
