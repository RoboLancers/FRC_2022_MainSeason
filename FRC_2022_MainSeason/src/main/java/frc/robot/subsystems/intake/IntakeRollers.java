package frc.robot.subsytems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

public class IntakeRollers {
    
    @Override
    public void setRollerMotorPower() {
        if(!isRetracted) {
            rollerMotor.set(Constants.Intake.ROLLER_MOTOR_POWER);
        }
    }
}