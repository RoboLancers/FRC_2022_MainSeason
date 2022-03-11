package frc.robot.subsystems.drivetrain.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class TurnToAngle extends PIDCommand {
    private Drivetrain drivetrain;

    public TurnToAngle(Drivetrain drivetrain, DoubleSupplier setpoint){
        super(
            new PIDController(
                Constants.Drivetrain.kP,
                Constants.Drivetrain.kI,
                Constants.Drivetrain.kD
            ),
            drivetrain::getHeading,
            setpoint,
            (outputPower) -> {
                drivetrain.leftMotors.set(outputPower);
                drivetrain.rightMotors.set(-outputPower);
            }
        );
        this.m_controller.setTolerance(Constants.Drivetrain.kMaxAbsoluteError);
        
        this.drivetrain = drivetrain;
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
