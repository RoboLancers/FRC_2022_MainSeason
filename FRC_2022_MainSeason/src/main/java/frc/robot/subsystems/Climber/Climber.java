package frc.robot.subsystems.climber;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    public CANSparkMax climberMotor1;
    public CANSparkMax climberMotor2;
    public RelativeEncoder climbEncoder1;
    public RelativeEncoder climbEncoder2;
    public boolean isEnabled;
    
    public Climber(){
        climberMotor1 = new CANSparkMax(Constants.Climber.CLIMBER_PORT1, CANSparkMax.MotorType.kBrushless);
        climberMotor2 = new CANSparkMax(Constants.Climber.CLIMBER_PORT2, CANSparkMax.MotorType.kBrushless);
        climbEncoder1 = climberMotor1.getEncoder();
        climbEncoder2 = climberMotor2.getEncoder();
        climbEncoder1.setPositionConversionFactor(Constants.Climber.kRotationToInch);
        climbEncoder2.setPositionConversionFactor(Constants.Climber.kRotationToInch);
        climbEncoder1.setPosition(0);
        climbEncoder2.setPosition(0);
        climberMotor1.setIdleMode(IdleMode.kBrake);
        climberMotor2.setIdleMode(IdleMode.kBrake);
        climberMotor1.setInverted(true);
        climberMotor2.setInverted(false);
       climberMotor1.setSoftLimit(SoftLimitDirection.kForward, Constants.Climber.kMaxHeight1);
       climberMotor2.setSoftLimit(SoftLimitDirection.kForward, Constants.Climber.kMaxHeight2);
        
    }
    public void setEncoderPosition(){
        climbEncoder1.setPosition(0);
        climbEncoder2.setPosition(0);
    }
    public void set(double power){
        climberMotor1.set(power);
        climberMotor2.set(power);
    }
    
    public double getPosition1(){
        return climbEncoder1.getPosition();
    }

    public double getPosition2(){
        return climbEncoder2.getPosition();
    }

    public boolean isHung(){
        if (this.climberMotor2.getOutputCurrent() >= Constants.Climber.kNormalHangCurrent){
            return true;
        }
        return false;
    }
}
