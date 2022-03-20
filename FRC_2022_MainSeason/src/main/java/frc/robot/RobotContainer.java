package frc.robot;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import frc.robot.subsystems.climber.Climber;
import com.kauailabs.navx.frc.AHRS;
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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drivetrain.Pneumatics;
import frc.robot.commands.MoveForward;
import frc.robot.commands.OneBallAuto;
import frc.robot.commands.TaxiAuto;
import frc.robot.commands.TwoBallAuto;
// import frc.robot.commands.GeneralizedReleaseRoutine;
import frc.robot.commands.UpdateLights;
import frc.robot.commands.ZeroClimber;
import frc.robot.subsystems.climber.commands.ManualClimber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GearShifter;
import frc.robot.subsystems.drivetrain.commands.ToggleGearShifter;
import frc.robot.subsystems.drivetrain.commands.TurnToAngle;
import frc.robot.subsystems.drivetrain.commands.UseCompressor;
//import frc.robot.subsystems.misc.AddressableLEDs;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.turret.Turret;
//import frc.robot.subsystems.turret.commands.ActiveLaunchTrajectory;
import frc.robot.subsystems.drivetrain.enums.GearShifterState;
import frc.robot.subsystems.misc.AddressableLEDs;
import frc.robot.subsystems.misc.Camera;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.UseIntake;
import frc.robot.subsystems.turret.LaunchTrajectory;
import frc.robot.subsystems.turret.commands.ActiveLaunchTrajectory;
import frc.robot.subsystems.turret.commands.LowHubShoot;
import frc.robot.subsystems.turret.commands.UpperHubShoot;
import frc.robot.subsystems.turret.commands.ZeroPitch;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.NotifierCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.util.XboxController;
import frc.robot.util.XboxController.Axis;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// import frc.robot.subsystems.climber.commands.ResetClimber;


public class RobotContainer {
  private RobotContainer m_robotContainer;

  /*   Controllers   */
  private final XboxController driverController = new XboxController(0);
  private final XboxController manipulatorController = new XboxController(1);

  /*   Subsystems   */
  private final Drivetrain drivetrain = new Drivetrain(driverController);
  private final Pneumatics pneumatics = new Pneumatics();
  private final GearShifter gearShifter = new GearShifter(pneumatics);
  private final Indexer indexer = new Indexer();
  private final Camera camera = new Camera();
  public final Turret turret = new Turret();
  private final Climber climber = new Climber();
  private final Intake intake = new Intake();
  // private AddressableLEDs m_AddressableLEDs = new AddressableLEDs();

  /*   Autonomous Trajectory   */
  private Trajectory trajectory = new Trajectory();
  private String trajectoryJSON = "pathplanner/generatedJSON/Test Path.wpilib.json";
  private AHRS gyro = new AHRS(SPI.Port.kMXP);
  private PIDController rightPID = new PIDController(Constants.Trajectory.kP, 0, 0);
  private PIDController leftPID = new PIDController(Constants.Trajectory.kP, 0, 0);
  private Field2d m_field = new Field2d();

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    this.configureButtonBindings();

    // A split-stick arcade command, with forward/backward controlled by the left hand, and turning controlled by the right.
    this.drivetrain.setDefaultCommand(
      new RunCommand(
        () -> {
          this.drivetrain.arcadeDrive(driverController.getAxisValue(XboxController.Axis.LEFT_Y), driverController.getAxisValue(XboxController.Axis.RIGHT_X));
        },
        drivetrain
      )
    );

    indexer.setDefaultCommand(new RunCommand(() -> {
      indexer.setPower(manipulatorController.getAxisValue(XboxController.Axis.RIGHT_Y));
    }, indexer));

    intake.setDefaultCommand(new RunCommand(() -> {
      intake.setPower(Math.signum(manipulatorController.getAxisValue(XboxController.Axis.RIGHT_TRIGGER)));
    }, intake));

    climber.setDefaultCommand(new ManualClimber(manipulatorController, climber));

    turret.setDefaultCommand(new ActiveLaunchTrajectory(turret));

    // Shot trajectory tuning

    SmartDashboard.putNumber("Angular kP", SmartDashboard.getNumber("Angular kP", Constants.Turret.Yaw.kP));
    SmartDashboard.putNumber("Angular kI", SmartDashboard.getNumber("Angular kI", Constants.Turret.Yaw.kI));
    SmartDashboard.putNumber("Angular kD", SmartDashboard.getNumber("Angular kD", Constants.Turret.Yaw.kD));

    SmartDashboard.putNumber("High Shot Speed", SmartDashboard.getNumber("High Shot Speed", 3700));
    SmartDashboard.putNumber("High Shot Angle", SmartDashboard.getNumber("High Shot Angle", 3.5));
  
    autoChooser.setDefaultOption("One Ball Auto", new OneBallAuto(drivetrain, turret, indexer));
    autoChooser.addOption("Two Ball Auto", new TwoBallAuto(drivetrain, turret, indexer, intake));
    autoChooser.addOption("Go Forth", new MoveForward(drivetrain, turret, indexer));
    autoChooser.addOption("Zero Climber", new ZeroClimber(climber, turret));
    autoChooser.addOption("Nothing", new ZeroPitch(turret));
  }

    // manipulatorController.whenPressed(XboxController.Trigger.RIGHT_TRIGGER, new GeneralizedReleaseRoutine(indexer, turret));
    // manipulatorController.whenPressed(XboxController.Trigger.RIGHT_TRIGGER, new GeneralizedReleaseRoutine(indexer, turret));
    // manipulatorController.whenPressed(XboxController.LEFT_BUMPER, new PassThrough Out);
    // manipulatorController.whenPressed(XboxController.Button.RIGHT_BUMPER, new RunCommand(
    // () -> {
    // indexer.setPower(Constants.Indexer.kIndexerSpeed), indexer;
    // }
    // );
    // manipulatorController.whenPressed(XboxController.Axis.LEFT_Y, new ManualClimber(driverController, climber));
    // manipulatorController.whenPressed(XboxController.Up, new REzero);
    // manipulatorController.whenPressed(XboxController.DOWN, new ShootFromLaunchpad);
    // manipulatorController.whenPressed(XboxController.Button.A, new ClimberDown);
    // manipulatorController.whenPressed(XboxController.ButtonThinggggg, new Instant)
    // manipulatorController.whenPressed(XboxController.Button.B, new SequentialCommandGroup(
    //   new ZeroAndDisable(turret),
    //   new UpClimber(climber, Constants.Climber.kLowClimb)));
    // manipulatorController.whenPressed(XboxController.Button.Y, new SequentialCommandGroup(
    //   new ZeroAndDisable(turret),
    //   new UpClimber(climber, Constants.Climber.kMidClimb)));
 
    private void configureButtonBindings(){
    manipulatorController.whenPressed(XboxController.Button.X, new ZeroPitch(turret));

    driverController.whileHeld(XboxController.Button.RIGHT_BUMPER, new TurnToAngle(drivetrain, turret, () -> {
      if(turret.limelight.hasTarget()){
        return turret.limelight.yawOffset() + drivetrain.getHeading();
      } else {
        return drivetrain.getHeading();
      }
    }));

    manipulatorController.whenPressed(XboxController.Button.B, new InstantCommand(intake::toggleDeploy));

    manipulatorController.whileHeld(XboxController.Button.RIGHT_BUMPER, new UpperHubShoot(turret));
    manipulatorController.whileHeld(XboxController.Button.LEFT_BUMPER, new LowHubShoot(turret));
  
    // manipulatorController.whenPressed(XboxController.Button.A, new ResetClimber(climber, climber.climberMotor1));
    // manipulatorController.whenPressed(XboxController.Button.B, new ResetClimber(climber, climber.climberMotor2));

    manipulatorController.whenPressed(XboxController.Button.Y, new InstantCommand(climber :: setEncoderPosition));
    manipulatorController.whenPressed(XboxController.POV.DOWN, new InstantCommand(climber :: setEncoderPosition));
  }

  public Command getAutonomousCommand() {
    // The voltage constraint makes sure the robot doesn't exceed a certain voltage during runtime.
    /*var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
          Constants.Trajectory.ksVolts, 
          Constants.Trajectory.ksVoltSecondsPerMeter,
          Constants.Trajectory.kaVoltSecondsSquaredPerMeter),
        Constants.Trajectory.kDriveKinematics,10);*/

    // Gives the trajectory the constants determined in characterization.
    /*TrajectoryConfig config = new TrajectoryConfig(
      Constants.Trajectory.kMaxSpeedMetersPerSecond,
      Constants.Trajectory.kMaxAccelerationMetersPerSecondSquared
    ).setKinematics(Constants.Trajectory.kDriveKinematics).addConstraint(autoVoltageConstraint);    

    // Generates the actual path with given points.
    /*
      trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
          // Pass through these two interior waypoints, making an 's' curve path
          List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
          // End 3 meters straight ahead of where we started, facing forward
          new Pose2d(3, 0, new Rotation2d(0)),
          // Pass config
          config);
    */
            
    /*try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON); 
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    // Ramsete is a trajectory tracker and auto corrector. We feed it parameters into a ramsete command
    // so that it constantly updates and corrects the trajectory auto.
   /* RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory, 
      drivetrain::getPose, // Gets the translational and rotational position of the robot.
      new RamseteController(Constants.Trajectory.kRamseteB, Constants.Trajectory.kRamseteZeta),//Uses constants of 2.0 and 0.7
      new SimpleMotorFeedforward( // Feedforward controller to control the motors before they move
        Constants.Trajectory.ksVolts,  
        Constants.Trajectory.ksVoltSecondsPerMeter,
        Constants.Trajectory.kaVoltSecondsSquaredPerMeter),
        Constants.Trajectory.kDriveKinematics, 
        drivetrain::getWheelSpeeds,
        leftPID,
        rightPID,
        drivetrain::tankDriveVolts,
        drivetrain);
    drivetrain.resetOdometry(trajectory.getInitialPose());
    return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0,0));
  */ return autoChooser.getSelected();
  //return new ShootTwoBalls(drivetrain, turret, indexer, intake); error: "unreachable code"
 }

 

 
   
    




  public void doSendables(){
    SmartDashboard.putNumber("Gyro Angle", drivetrain.getHeading());
    SmartDashboard.putData(autoChooser);
    SmartDashboard.putNumber("Actual Speed", turret.flywheel.getVelocity());
    SmartDashboard.putNumber("Actual Pitch", turret.pitch.getPosition());
    SmartDashboard.putNumber("Climber Encoder1", climber.climbEncoder1.getPosition());
    SmartDashboard.putNumber("Climber Encoder2", climber.climbEncoder2.getPosition());
    SmartDashboard.putNumber("Climber Current1", climber.climberMotor1.getOutputCurrent());
    SmartDashboard.putNumber("Climber Current1", climber.climberMotor1.getOutputCurrent());

  }
}