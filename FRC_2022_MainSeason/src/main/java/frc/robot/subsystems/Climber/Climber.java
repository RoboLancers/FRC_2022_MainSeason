package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Climber {
    public CANSparkMax Climber;
    public void Climber(){
        Climber = new CANSparkMax(0, MotorType.kBrushless);
    }
    public void set(double power) {
         Climber.set(power);
    }
    
}
}