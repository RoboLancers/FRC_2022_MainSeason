package frc.robot.subsystems.Climber;
import com.revrobotics.CANSparkMax;

public class Climber{
    public CANSparkMax Climber;

    public Climber(){
        Climber = new CANSparkMax(port, CANSparkMax.MotorType.kBrushless);

    }
    public void set(double power){
        Climber.set(power);
        
    }
}
