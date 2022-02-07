package frc.robot.subsystems.turret;


import frc.robot.Constants;
import com.revrobotics.CANSparkMax;

public class FlyWheel {

    CANSparkMax flyWheel = new CANSparkMax(Constants.Turret.Ports.FLYWHEEL_PORT, CANSparkMax.MotorType.kBrushless);
    CANSparkMax kickWheel = new CANSparkMax(Constants.Turret.Ports.KICKWHEEL_PORT, CANSparkMax.MotorType.kBrushless);

    public double FlywheelSpeed(){
        return flyWheel.getEncoder().getVelocity(); //should return in ticks per second
    }

}