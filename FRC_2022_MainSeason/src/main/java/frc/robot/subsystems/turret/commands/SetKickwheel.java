//Nailah 
package frc.robot.subsystems.turret.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.SparkMaxWrapper;

public class SetKickwheel extends PIDCommand {

    private static double velocityErrorThreshold = 0.0; //figure out value
    
    public SetKickwheel (double angle, double velocity, Turret turret ) {
        super(
            new PIDController (
                Constants.Turret.TunedCoefficients.KickwheelPID.p,
                Constants.Turret.TunedCoefficients.KickwheelPID.i,
                Constants.Turret.TunedCoefficients.KickwheelPID.d
            ),
            () -> {

                return (kickwheelMotorX.getVelocity() + kickwheelMotorY.getVelocity()) / 2; //average velocity?

            }, 

            () -> {
                kickwheelMotorX.getVelocity(output);
                kickwheelMotorY.getVelocity(output);
            }


        );

        this.getController().setTolerance(velocityErrorThreshold);
        
    }

    @Override 
    public void end(boolean interrupted) {

    };

    @Override
    public boolean isFinished() {
        return(getController().atSetpoint());
    }

    
}
