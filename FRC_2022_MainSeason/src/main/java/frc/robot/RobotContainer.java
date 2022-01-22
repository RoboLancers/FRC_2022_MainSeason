package frc.robot;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class RobotContainer {
  // Subsystems and commands
  Drivetrain drivetrain = new drivetrain();
  Button button = new Button();
  button.toggleWhenPressed(new StartEndCommand(drivetrain::onMethod,
  drivetrain::offMethod,
  drivetrain));


  button.whileHeld(new StartendCommand(drivetrain::onMethod,
  drivetrain::offMethod,
  drivetrain));

  
  Xboxcontroller xboxcontroller = new Xboxcontroller(1);
  //create class for xbox controller 
  

  joystickButton.Button.A, new joystickButton(joystick, 1);

  buttonA.whenPressed(new intake());
  
  public RobotContainer() {
    configureButtonBindings();

  }

  private void configureButtonBindings() {}
}