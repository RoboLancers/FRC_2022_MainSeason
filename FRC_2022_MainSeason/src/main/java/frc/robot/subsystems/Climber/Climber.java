package frc.robot.subsystems.climber;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    public CANSparkMax climberMotor1;
    public CANSparkMax climberMotor2;
    public RelativeEncoder climbEncoder;

    public Climber(){
        climberMotor1 = new CANSparkMax(Constants.Climber.CLIMBER_PORT, CANSparkMax.MotorType.kBrushless);
        climbEncoder = climberMotor1.getEncoder();
        climbEncoder.setPositionConversionFactor(Constants.Climber.kRotationToInch);
        climberMotor2.follow(climberMotor1);
        climberMotor1.setIdleMode(IdleMode.kBrake);
    }

    public void set(double power){
        climberMotor1.set(power);
        }
    
    public double getPosition(){
        return climbEncoder.getPosition();
    }

    public boolean isHung(){
        if (this.climberMotor1.getOutputCurrent() >= Constants.Climber.kNormalHangCurrent){
            return true;
        }
        else{
            return false;
        } 
    }
}
