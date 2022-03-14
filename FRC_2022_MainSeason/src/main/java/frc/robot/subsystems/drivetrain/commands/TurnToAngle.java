package frc.robot.subsystems.drivetrain.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class TurnToAngle extends PIDCommand {
    private Drivetrain drivetrain;

    public TurnToAngle(Drivetrain drivetrain, DoubleSupplier setpoint){
        super(
            new PIDController(
                Constants.Turret.Yaw.kP,
                Constants.Turret.Yaw.kI,
                Constants.Turret.Yaw.kD
            ),
            drivetrain::getHeading,
            setpoint,
            (outputPower) -> {
                drivetrain.leftMotors.set(outputPower);
                drivetrain.rightMotors.set(-outputPower);
            }
        );
        this.m_controller.setTolerance(Constants.Turret.Yaw.kErrorThreshold);
        
        this.drivetrain = drivetrain;
    }

    @Override
    public void execute(){
        this.m_controller.setP(SmartDashboard.getNumber("Angular kP", 0.0));
        this.m_controller.setI(SmartDashboard.getNumber("Angular kI", 0.0));
        this.m_controller.setD(SmartDashboard.getNumber("Angular kD", 0.0));
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.leftMotors.set(0);
        drivetrain.rightMotors.set(0);
    }

    @Override
    public boolean isFinished(){
        return this.m_controller.atSetpoint();
    }
}
