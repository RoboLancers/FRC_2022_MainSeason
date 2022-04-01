package frc.robot.subsystems.drivetrain.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.XboxController;

public class TurnToAngleDrive extends PIDCommand {
        private Drivetrain drivetrain;  
    
        public TurnToAngleDrive(Drivetrain drivetrain, XboxController driverController, DoubleSupplier setpoint){
            super(
                new PIDController(
                    Constants.Turret.Yaw.kP,
                    Constants.Turret.Yaw.kI,
                    Constants.Turret.Yaw.kD
                    // SmartDashboard.getNumber("Turn kP", 0),
                    // SmartDashboard.getNumber("Turn kI", 0),
                    // SmartDashboard.getNumber("Turn kD", 0)

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
                    drivetrain.arcadeDrive(driverController.getAxisValue(XboxController.Axis.LEFT_Y), outputPower, false);
                },
                drivetrain
            );
            this.getController().setTolerance(Constants.Turret.Yaw.kErrorThreshold);
            // this.getController().setTolerance(SmartDashboard.getNumber("Turn error max", 0));
            this.getController().enableContinuousInput(-180.0, 180.0);

            SmartDashboard.putBoolean("Angular Running", true);
    
            this.drivetrain = drivetrain;

            addRequirements(drivetrain);
        }
    
        @Override
        public void end(boolean interrupted){
            SmartDashboard.putBoolean("Angular Running", false);
            drivetrain.leftMotors.set(0);
            drivetrain.rightMotors.set(0);
        }
    
        @Override
        public boolean isFinished(){
            // return true;
            return this.getController().atSetpoint();
        }
}