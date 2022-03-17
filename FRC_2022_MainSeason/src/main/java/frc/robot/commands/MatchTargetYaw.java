package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.turret.Turret;

public class MatchTargetYaw extends PIDCommand {
    private Drivetrain drivetrain;

    public MatchTargetYaw(Drivetrain drivetrain, Turret turret){
        super(
            new PIDController(
                SmartDashboard.getNumber("Angular kP", 0),
                SmartDashboard.getNumber("Angular kI", 0),
                SmartDashboard.getNumber("Angular kD", 0)
            ),
            () -> {
                if(turret.limelight.hasTarget()){
                    return turret.limelight.yawOffset();
                } else {
                    return 0;
                }
            },
            () -> 0,
            (outputPower) -> {
                drivetrain.leftMotors.set(outputPower);
                drivetrain.rightMotors.set(-outputPower);
            },
            drivetrain
        );
        this.m_controller.setTolerance(Constants.Turret.Yaw.kErrorThreshold);

        this.drivetrain = drivetrain;
    }

    @Override
    public void execute(){
        this.m_controller.setPID(
            SmartDashboard.getNumber("Angular kP", 0.0),
            SmartDashboard.getNumber("Angular kI", 0.0),
            SmartDashboard.getNumber("Angular kD", 0.0)
        );
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
