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
import frc.robot.subsystems.turret.commands.ActiveFlywheel;
import frc.robot.subsystems.turret.commands.ActiveLaunchTrajectory;
import frc.robot.subsystems.turret.commands.ActivePitch;
import frc.robot.subsystems.turret.commands.LowHubShot;
import frc.robot.subsystems.turret.commands.UpperHubShot;
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
    this.drivetrain.setDefaultCommand(new RunCommand(
      () -> this.drivetrain.arcadeDrive(driverController.getAxisValue(XboxController.Axis.LEFT_Y), driverController.getAxisValue(XboxController.Axis.RIGHT_X)),
      drivetrain
    ));

    indexer.setDefaultCommand(new RunCommand(() -> {
      indexer.setPower(manipulatorController.getAxisValue(XboxController.Axis.RIGHT_Y));
    }, indexer));

    intake.setDefaultCommand(new RunCommand(() -> {
      intake.setPower(Math.signum(manipulatorController.getAxisValue(XboxController.Axis.RIGHT_TRIGGER)));
    }, intake));

    climber.setDefaultCommand(new ManualClimber(manipulatorController, climber));

    turret.setDefaultCommand(new ActiveLaunchTrajectory(turret));
    turret.flywheel.setDefaultCommand(new ActiveFlywheel(turret));
    turret.pitch.setDefaultCommand(new ActivePitch(turret));

    SmartDashboard.putNumber("High Shot Speed", SmartDashboard.getNumber("High Shot Speed", 3700));
    SmartDashboard.putNumber("High Shot Angle", SmartDashboard.getNumber("High Shot Angle", 3.5));
  
    autoChooser.setDefaultOption("One Ball Auto", new OneBallAuto(drivetrain, turret, indexer));
    autoChooser.addOption("Two Ball Auto", new TwoBallAuto(drivetrain, turret, indexer, intake));
    autoChooser.addOption("Go Forth", new MoveForward(drivetrain, turret, indexer));
    autoChooser.addOption("Zero Climber", new ZeroClimber(climber, turret));
    autoChooser.addOption("Nothing", new ZeroPitch(turret));
  }
 
  private void configureButtonBindings(){
    driverController.whileHeld(XboxController.Button.RIGHT_BUMPER, new TurnToAngle(drivetrain, turret, () -> {
      return drivetrain.getHeading() + (turret.limelight.hasTarget() ? turret.limelight.yawOffset() : 0);
    }));

    manipulatorController
      .whenPressed(XboxController.Button.B, new InstantCommand(intake::toggleDeploy))
      .whenPressed(XboxController.Button.X, new ZeroPitch(turret))
      // .whenPressed(XboxController.Button.Y, new InstantCommand(climber::resetEncoderPosition))
      // .whenPressed(XboxController.Button.A, new ResetClimber(climber, climber.climberMotor1))
      // .whenPressed(XboxController.Button.B, new ResetClimber(climber, climber.climberMotor2))
      .whileHeld(XboxController.Button.LEFT_BUMPER, new LowHubShot(turret))
      .whileHeld(XboxController.Button.RIGHT_BUMPER, new UpperHubShot(turret));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void doSendables(){
    SmartDashboard.putData(autoChooser);
    SmartDashboard.putNumber("Gyro Angle", drivetrain.getHeading());
    SmartDashboard.putNumber("Actual Speed", turret.flywheel.getVelocity());
    SmartDashboard.putNumber("Actual Pitch", turret.pitch.getPosition());
    SmartDashboard.putNumber("Climber Encoder1", climber.climbEncoder1.getPosition());
    SmartDashboard.putNumber("Climber Encoder2", climber.climbEncoder2.getPosition());
    SmartDashboard.putNumber("Climber Current1", climber.climberMotor1.getOutputCurrent());
    SmartDashboard.putNumber("Climber Current1", climber.climberMotor1.getOutputCurrent());
  }
}