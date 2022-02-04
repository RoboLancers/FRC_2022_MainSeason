package frc.robot.subsytems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;
import frc.robot.Variables;

public class IntakeRollers {
    
    @Override
    public void setRollerMotorPower() {
        if(!Variables.Intake.retracted) {
            rollerMotor.set(Constants.Intake.ROLLER_MOTOR_POWER);
        }
        else {
            rollerMotor.set(Constants.Intake.ROLLER_MOTOR_OFF)
        }
    }
}