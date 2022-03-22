package frc.robot.subsystems.turret.subsystems;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

public class TurretFlywheel extends SubsystemBase {
    private CANSparkMax leadingMotor;
    private CANSparkMax followerMotor;
    
    private RelativeEncoder encoder;

    public TurretFlywheel(){
        this.leadingMotor = new CANSparkMax(Constants.Turret.Ports.kFlywheelMotorA, CANSparkMax.MotorType.kBrushless);
        this.followerMotor = new CANSparkMax(Constants.Turret.Ports.kFlywheelMotorB, CANSparkMax.MotorType.kBrushless);

        this.leadingMotor.setInverted(false);
        this.followerMotor.setInverted(true);
        this.leadingMotor.setIdleMode(IdleMode.kCoast);
        this.followerMotor.setIdleMode(IdleMode.kCoast);

        this.followerMotor.follow(this.leadingMotor);
        
        this.encoder = this.leadingMotor.getEncoder();
        this.encoder.setPosition(0.0);
        this.encoder.setVelocityConversionFactor(1);
    }

    public void setVoltage(double voltage){
        this.leadingMotor.setVoltage(voltage);
    }

    public double getVelocity(){
        return this.encoder.getVelocity();
    }

    public boolean isUpToSpeed(double velocitySetpoint){
        return Math.abs(this.getVelocity() - velocitySetpoint) < Constants.Turret.Flywheel.kErrorThreshold;
    }
}