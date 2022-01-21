package frc.robot.subsystems.drivetrain;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.enums.GearShifterState;


public class Drivetrain extends SubsystemBase {
    public final DifferentialDrive difDrive;

        //Drivetrain
        public Drivetrain(){
            CANSparkMax leftMotorFront = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
            CANSparkMax leftMotorMid = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
            CANSparkMax leftMotorBack = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
            MotorControllerGroup left = new MotorControllerGroup(leftMotorFront, leftMotorMid, leftMotorBack);

            CANSparkMax rightMotorFront = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
            CANSparkMax rightMotorMid = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
            CANSparkMax rightMotorBack = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
            MotorControllerGroup right = new MotorControllerGroup(rightMotorFront, rightMotorMid, rightMotorBack);
            
            right.setInverted(true);

            difDrive = new DifferentialDrive(left, right);
        
            /*add in accerleration limiter 
            list motors
            
            */  
        }

        public void arcadeDrive(double throttle,double turn) {
            difDrive.arcadeDrive(throttle,turn);
        }


    
}
