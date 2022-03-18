package frc.robot.subsystems.drivetrain.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.turret.Turret;

public class TurnToAngle extends PIDCommand {
        private Drivetrain drivetrain;
    
        public TurnToAngle(Drivetrain drivetrain, Turret turret, DoubleSupplier setpoint){
            super(
                new PIDController(
                    SmartDashboard.getNumber("Angular kP", 0),
                    SmartDashboard.getNumber("Angular kI", 0),
                    SmartDashboard.getNumber("Angular kD", 0)
                ),
                drivetrain::getHeading,
                () -> {
                    double setpointValue = setpoint.getAsDouble();
                    SmartDashboard.putNumber("Angular Setpoint", setpointValue);
                    SmartDashboard.putNumber("Angular Error", setpointValue - drivetrain.getHeading());
                    return setpointValue;
                },
                (outputPower) -> {
                    SmartDashboard.putNumber("Angular Output", outputPower);
                    drivetrain.leftMotors.set(outputPower);
                    drivetrain.rightMotors.set(-outputPower);
                },
                drivetrain
            );
            this.getController().setTolerance(Constants.Turret.Yaw.kErrorThreshold);
            this.getController().enableContinuousInput(-180.0, 180.0);

            SmartDashboard.putBoolean("Angular Running", true);
    
            this.drivetrain = drivetrain;
        }
    
        // @Override
        // public void execute(){
        //     this.getController().setPID(
        //         SmartDashboard.getNumber("Angular kP", 0.0),
        //         SmartDashboard.getNumber("Angular kI", 0.0),
        //         SmartDashboard.getNumber("Angular kD", 0.0)
        //     );
        // }
    
        @Override
        public void end(boolean interrupted){
            SmartDashboard.putBoolean("Angular Running", false);
            drivetrain.leftMotors.set(0);
            drivetrain.rightMotors.set(0);
        }
    
        @Override
        public boolean isFinished(){
            return this.getController().atSetpoint();
        }
}