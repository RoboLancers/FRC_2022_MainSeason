package frc.robot.subsystems.turret;

import com.qualcomm.robotcore.hardware;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Flywheel{

CANSparkMax flyWheel = new CANSparkMax(CANSparkMaxLowLevel.MotorType.kBrushless);
CANSparkMax kickWheel = new CANSparkMax(CANSparkMaxLowLevel.MotorType.kBrushless);
CANEncoder flyEncoder = flyWheel.getEncoder();

public double FlywheelSpeed(){
double speed = flyEnocder.getVelocity(); //should return in ticks per second
return speed;
}

}



