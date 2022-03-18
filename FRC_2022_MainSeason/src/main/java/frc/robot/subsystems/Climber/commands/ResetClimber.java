package frc.robot.subsystems.climber.commands;
import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ResetClimber extends CommandBase{
    private Climber climber;
    private CANSparkMax climberMotor1;
    
    public ResetClimber(Climber climber, CANSparkMax climberMotor1 ) {
        this.climber = climber;
        this.climberMotor1 = climberMotor1;
    }

    @Override
    public void execute(){
        climberMotor1.set(Constants.Climber.kNegativePower*0.3);

    }

    @Override
    public boolean isFinished(){
        return (climberMotor1.getOutputCurrent() >= Constants.Climber.kResetCurrent);
    }
    
    @Override
    public void end(boolean interrupted){
        climber.climberMotor1.set(0);
        climber.climberMotor2.set(0);
    }
}