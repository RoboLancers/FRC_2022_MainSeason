package frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class RobotContainer {
  // Subsystems and commands

  Controller driverController = new Controller(0);
  Controller manipulatorController = new Controller(1);
  //create class for xbox controller 
  

  
  
  public RobotContainer() {
    configureButtonBindings();

  }

  private void configureButtonBindings() {}
}