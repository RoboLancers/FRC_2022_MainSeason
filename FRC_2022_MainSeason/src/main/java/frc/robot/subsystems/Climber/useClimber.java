package frc.robot.subsystem.Climber;

public class UseClimber {
    public static XboxController manipulatorXboxController = new XboxController(1, 0.2);
    public void configureButtonBinding(){
        manipulatorXboxController
          //.whenPressed(XboxController.Button.Y, new useClimber(Climber))
            .whileheld(XboxController.Joystick.Left, new useClimber(Climber))
    }
}
