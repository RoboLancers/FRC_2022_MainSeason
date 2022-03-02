package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.XboxController;

// A split-stick arcade command, with forward/backward controlled by the left hand, and turning controlled by the right.
public class TeleopDrive extends CommandBase {
    private Drivetrain driveTrain;
    private XboxController driverController;

    public TeleopDrive(Drivetrain driveTrain, XboxController driverController){
        this.driveTrain = driveTrain;
        this.driverController = driverController;
        addRequirements(driveTrain);
    }

    @Override
    public void execute(){
        this.driveTrain.arcadeDrive(
            driverController.getAxisValue(XboxController.Axis.LEFT_Y),
            driverController.getAxisValue(XboxController.Axis.RIGHT_X));
    }
}
