package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.MoveForward;
import frc.robot.commands.OneBallAuto;
import frc.robot.commands.TwoBallAuto;
import frc.robot.commands.ZeroClimber;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.commands.ManualClimber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GearShifter;
import frc.robot.subsystems.drivetrain.Pneumatics;
import frc.robot.subsystems.drivetrain.commands.TurnToAngle;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.misc.Camera;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.ActiveLaunchTrajectory;
import frc.robot.subsystems.turret.commands.LowHubShoot;
import frc.robot.subsystems.turret.commands.UpperHubShoot;
import frc.robot.subsystems.turret.commands.ZeroPitch;
import frc.robot.util.XboxController;

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
    drivetrain));

    indexer.setDefaultCommand(new RunCommand(() -> {
      indexer.setPower(Math.signum(manipulatorController.getAxisValue(XboxController.Axis.RIGHT_Y)));
    }, indexer));

    intake.setDefaultCommand(new RunCommand(() -> {
      intake.setPower(Math.signum((manipulatorController.getAxisValue(XboxController.Axis.RIGHT_TRIGGER))));
    }, intake));

    climber.setDefaultCommand(new ManualClimber(manipulatorController, climber));

    turret.setDefaultCommand(new ActiveLaunchTrajectory(turret));

    // Shot trajectory tuning

    SmartDashboard.putBoolean("Use Interpolation", SmartDashboard.getBoolean("Use Interpolation", true));

    SmartDashboard.putNumber("High Shot Speed", SmartDashboard.getNumber("High Shot Speed", 3700));
    SmartDashboard.putNumber("High Shot Angle", SmartDashboard.getNumber("High Shot Angle", 3.5));
  
    autoChooser.setDefaultOption("One Ball Auto", new OneBallAuto(drivetrain, turret, indexer));
    autoChooser.addOption("Two Ball Auto", new TwoBallAuto(drivetrain, turret, indexer, intake));
    autoChooser.addOption("Go Forth", new MoveForward(drivetrain, turret, indexer));
    autoChooser.addOption("Zero Climber", new ZeroClimber(climber, turret));
    autoChooser.addOption("Nothing", new ZeroPitch(turret));
  }
 
  private void configureButtonBindings(){
    manipulatorController.whenPressed(XboxController.Button.X, new ZeroPitch(turret));

    driverController.whileHeld(XboxController.Button.RIGHT_BUMPER, new TurnToAngle(drivetrain, turret, () -> {
      return drivetrain.getHeading() + (turret.limelight.hasTarget() ? turret.limelight.yawOffset() : 0);
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
    return autoChooser.getSelected();
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
    SmartDashboard.putNumber("Indexer Speed", indexer.indexerMotor.get());
  }
}