package frc.robot.subsystems.turret;

com.qualcomm.robotcore.hardware
edu.wpi.first.wpilibj.Encoder
edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax


CANSparkMax flyWheel = new CANSparkMax(CANSparkMaxLowLevel.MotorType.kBrushless);
CANSparkMax kickWheel = new CANSparkMax(CANSparkMaxLowLevel.MotorType.kBrushless);
CANEncoder flyEncoder = flyWheel.getEncoder();

public class Flywheel{



public double FlywheelSpeed(){
double speed = flyEnocder.getVelocity(); //should return in ticks per second
return speed;
}

}



