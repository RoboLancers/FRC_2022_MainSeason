// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drivetrain.Pneumatics;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GearShifter;
import frc.robot.subsystems.drivetrain.commands.UseCompressor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Drivetrain dt = new Drivetrain();
  private Trajectory trajectory = new Trajectory();
  private String trajectoryJSON = "pathplanner/generatedJSON/Test Path.wpilib.json";
  private XboxController driverController = new XboxController(0);
  private PIDController rightPID= new PIDController(Constants.Trajectory.kP, 0, 0);
  private PIDController leftPID= new PIDController(Constants.Trajectory.kP, 0, 0);
  private Pneumatics pneumatics = new Pneumatics();
  public GearShifter gearshifter = new GearShifter();
 
  

  
  
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    pneumatics.setDefaultCommand(new UseCompressor(pneumatics));
 
    configureButtonBindings();
    dt.setDefaultCommand(
      // A split-stick arcade command, with forward/backward controlled by the left
      // hand, and turning controlled by the right.
      new RunCommand(
          () ->
              dt.arcadeDrive(-driverController.getLeftY(), driverController.getRightX()),
          dt));
}
   

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Toggle Gearshift
    new JoystickButton(driverController, Button.kA.value)
        .whenPressed(gearshifter::ToggleGearShifter);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //The voltage constraint makes sure the robot doesn't exceed a certain voltage during runtime.
    var autoVoltageConstraint = 
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
          Constants.Trajectory.ksVolts, 
          Constants.Trajectory.ksVoltSecondsPerMeter,
          Constants.Trajectory.kaVoltSecondsSquaredPerMeter),
        Constants.Trajectory.kDriveKinematics, 10);

    //Gives the trajectory the constants determined in characterization.
    TrajectoryConfig config = 
      new TrajectoryConfig(
        Constants.Trajectory.kMaxSpeedMetersPerSecond,
        Constants.Trajectory.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(Constants.Trajectory.kDriveKinematics )
        .addConstraint(autoVoltageConstraint);    

    //Generates the actual path with given points.
    /*trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);*/
    
    
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON); 

      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
   
    



    //Ramsete is a trajectory tracker and auto corrector. We feed it parameters into a ramsete command
    //so that it constantly updates and corrects the trajectory auto.
    RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory, 
      dt::getPose, //Gets the translational and rotational position of the robot.
        new RamseteController(Constants.Trajectory.kRamseteB, Constants.Trajectory.kRamseteZeta),//Uses constants of 2.0 and 0.7
        new SimpleMotorFeedforward( //Feedforward controller to control the motors before they move
          Constants.Trajectory.ksVolts,  
          Constants.Trajectory.ksVoltSecondsPerMeter,
          Constants.Trajectory.kaVoltSecondsSquaredPerMeter),
          Constants.Trajectory.kDriveKinematics, 
          dt::getWheelSpeeds,
          leftPID,
          rightPID,
          dt::tankDriveVolts,
          dt);

    dt.resetOdometry(trajectory.getInitialPose());
    return null;
    //return ramseteCommand.andThen(() -> dt.tankDriveVolts(0,0));
  }

  public void doSendables() {
    SmartDashboard.putNumber("Encoder", dt.getAverageEncoderDistance());
    SmartDashboard.putNumber("Heading", dt.getHeading());
    SmartDashboard.putNumber("Left Voltage", dt.getLeftVoltage());
    SmartDashboard.putNumber("Right Voltage", dt.getRightVoltage());
    SmartDashboard.putData("Right PID Controller",  rightPID);
    SmartDashboard.putData("Left PID Controller", leftPID);
    //Are these encoder positions correct?
    SmartDashboard.putNumber("Left Position", Constants.Trajectory.kDistPerRot * dt.getLeftEncoder().getPosition() / 42);
    SmartDashboard.putNumber("Right Position", Constants.Trajectory.kDistPerRot * dt.getRightEncoder().getPosition() / 42);
    
    SmartDashboard.putNumber("Left Encoder Ticks", dt.getLeftEncoder().getPosition() * 4096);
    SmartDashboard.putNumber("Right Encoder Ticks", dt.getRightEncoder().getPosition() * 4096);
    

    
  }

  
 
}
