package frc.robot;

import java.util.List;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.commands.LowRung;
import frc.robot.subsystems.climber.commands.MidRung;
import frc.robot.subsystems.climber.commands.UseClimber;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drivetrain.Pneumatics;
import frc.robot.commands.GeneralizedReleaseRoutine;
import frc.robot.subsystems.climber.commands.LowRung;
import frc.robot.subsystems.climber.commands.MidRung;
import frc.robot.commands.UpdateLights;
import frc.robot.subsystems.climber.Climber;
import frc.robot.commands.UpdateLights;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GearShifter;
import frc.robot.subsystems.drivetrain.commands.ToggleGearShifter;
import frc.robot.subsystems.drivetrain.commands.UseCompressor;
import frc.robot.subsystems.misc.AddressableLEDs;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.ActiveLaunchTrajectory;
import frc.robot.subsystems.turret.subsystems.yaw.commands.MatchHeadingYaw;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.util.XboxController;

public class RobotContainer {
  private final Drivetrain driveTrain = new Drivetrain();
  private final Indexer indexer = new Indexer();
  // private final Turret turret = new Turret();
  // private final Climber climber = new Climber();

  private Trajectory trajectory = new Trajectory();
  // private String trajectoryJSON = "paths/MyPath.wpilib.json";
  private RobotContainer m_robotContainer;
  private XboxController xboxController = new XboxController(0);
  private PIDController rightPID= new PIDController(Constants.Trajectory.kP, 0, 0);
  private PIDController leftPID= new PIDController(Constants.Trajectory.kP, 0, 0);
  private Field2d m_field = new Field2d();
  private Pneumatics pneumatics = new Pneumatics();
  private XboxController driverController = new XboxController(0);
  private XboxController manipulatorController = new XboxController(1);
  private GearShifter gearShifter;
  //private AddressableLEDs m_AddressableLEDs = new AddressableLEDs();

  public RobotContainer() {
    this.pneumatics.setDefaultCommand(new UseCompressor(pneumatics));

    this.indexer.setDefaultCommand(new RunCommand(
      () -> {
        if(this.indexer.balls[0] == null) {
          this.indexer.indexerMotor.set(Constants.Indexer.kStandardIndexerSpeed);
        } else {
          this.indexer.indexerMotor.set(Constants.Indexer.kIndexerOff);
        }
      }, this.indexer
    ));
    
    this.configureButtonBindings();

    // A split-stick arcade command, with forward/backward controlled by the left hand, and turning controlled by the right.
    this.driveTrain.setDefaultCommand(
      new RunCommand(
        () -> {
          this.driveTrain.arcadeDrive(-driverController.getAxisValue(XboxController.Axis.LEFT_Y), driverController.getAxisValue(XboxController.Axis.RIGHT_X));
        },
        driveTrain
      )
    );

    //m_AddressableLEDs.setDefaultCommand(new UpdateLights(turret, climber, indexer));
    
    //turret.setDefaultCommand(new ActiveLaunchTrajectory(turret));
    //turret.yaw.setDefaultCommand(new MatchHeadingYaw(turret.yaw));
    // m_AddressableLEDs.setDefaultCommand(new UpdateLights(turret, climber, indexer));
    
    // turret.setDefaultCommand(new ActiveLaunchTrajectory(turret));
    // turret.yaw.setDefaultCommand(new MatchHeadingYaw(turret.yaw));
  }

  private void configureButtonBindings() {
    driverController.whenPressed(XboxController.Button.A, new ToggleGearShifter(gearShifter));
    // driverController.whenPressed(XboxController.RIGHT_BUMPER, new Intake);
    // driverController.whenPressed(XboxController.Button.B, new Intake Deploy);
    // driverController.whenPressed(XboxController.UP, new HighGear);
    // driverController.whenPressed(XboxController.down, new LowGear);
    // driverController.whenPressed(XboxController.RIGHT_BUMPER, new Intake);

    //manipulatorController.whenPressed(XboxController.Trigger.RIGHT_TRIGGER, new GeneralizedReleaseRoutine(indexer, turret));
    // manipulatorController.whenPressed(XboxController.Trigger.RIGHT_TRIGGER, new GeneralizedReleaseRoutine(indexer, turret));
    // manipulatorController.whenPressed(XboxController.LEFT_BUMPER, new PassThrough Out);
    // manipulatorController.whenPressed(XboxController.RIGHT_BUMPER, new Passthrough In);
    // manipulatorController.whenPressed(XboxController.LEFT_JOYSTICK_BUTTON, new ManualControlClimber);
    // manipulatorController.whenPressed(XboxController.Up, new REzero);
    // manipulatorController.whenPressed(XboxController.DOWN, new ShootFromLaunchpad);
    // manipulatorController.whenPressed(XboxController.Button.A, new ClimberDown);
    //manipulatorController.whenPressed(XboxController.Button.B, new LowRung(climber,Constants.Climber.kLowClimb));
    //manipulatorController.whenPressed(XboxController.Button.Y, new MidRung(climber,Constants.Climber.kMidClimb));
    // manipulatorController.whenPressed(XboxController.Button.B, new LowRung(climber,Constants.Climber.kLowClimb));
    // manipulatorController.whenPressed(XboxController.Button.Y, new MidRung(climber,Constants.Climber.kMidClimb));
  }

  public Command getAutonomousCommand() {
    // The voltage constraint makes sure the robot doesn't exceed a certain voltage during runtime.
    var autoVoltageConstraint =  new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
        Constants.Trajectory.ksVolts, 
        Constants.Trajectory.ksVoltSecondsPerMeter,
        Constants.Trajectory.kaVoltSecondsSquaredPerMeter
      ),
      Constants.Trajectory.kDriveKinematics, 10
    );

    // Gives the trajectory the constants determined in characterization.
    TrajectoryConfig config = new TrajectoryConfig(
      Constants.Trajectory.kMaxSpeedMetersPerSecond,
      Constants.Trajectory.kMaxAccelerationMetersPerSecondSquared
    )
      .setKinematics(Constants.Trajectory.kDriveKinematics)
      .addConstraint(autoVoltageConstraint);    

    // Generates the actual path with given points.
    trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(
        new Translation2d(1, 1),
        new Translation2d(2, -1)
      ),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(3, 0, new Rotation2d(0)),
      // Pass config
      config
    );
    
    /*
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON); 

      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    */

    // Ramsete is a trajectory tracker and auto corrector. We feed it parameters into a ramsete command
    // so that it constantly updates and corrects the trajectory auto.
    RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory, 
      driveTrain::getPose, // Gets the translational and rotational position of the robot.
      new RamseteController(Constants.Trajectory.kRamseteB, Constants.Trajectory.kRamseteZeta),//Uses constants of 2.0 and 0.7
      new SimpleMotorFeedforward( // Feedforward controller to control the motors before they move
        Constants.Trajectory.ksVolts,  
        Constants.Trajectory.ksVoltSecondsPerMeter,
        Constants.Trajectory.kaVoltSecondsSquaredPerMeter),
        Constants.Trajectory.kDriveKinematics, 
        driveTrain::getWheelSpeeds,
        leftPID,
        rightPID,
        driveTrain::tankDriveVolts,
        driveTrain
    );

    driveTrain.resetOdometry(trajectory.getInitialPose());
    return ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0,0));
  }

  public void updateSmartDashboard(){
    SmartDashboard.putNumber("Encoder", driveTrain.getAverageEncoderDistance());
    SmartDashboard.putNumber("Heading", driveTrain.getHeading());
    SmartDashboard.putNumber("Left Voltage", driveTrain.getLeftVoltage());
    SmartDashboard.putNumber("Right Voltage", driveTrain.getRightVoltage());
    SmartDashboard.putData("Right PID Controller",  rightPID);
    SmartDashboard.putData("Left PID Controller", leftPID);
    SmartDashboard.putNumber("Left Position", Constants.Trajectory.kDistPerRot * driveTrain.getLeftEncoder().getPosition() / 42);
    SmartDashboard.putNumber("Right Position", Constants.Trajectory.kDistPerRot * driveTrain.getRightEncoder().getPosition() / 42);
  }
}
