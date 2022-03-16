package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.misc2022.EnhancedJoystick;
import frc.robot.commands.Angle_Idle;
import frc.robot.commands.Climb_CalibrateArmNoSwitch;
import frc.robot.commands.Climb_FirstBar;
import frc.robot.commands.Climb_LiftArm;
import frc.robot.commands.Climb_NextBar;
import frc.robot.commands.AutoAtGoal;
import frc.robot.commands.AutoGrabShootBall;
import frc.robot.commands.Auto_DriveThreeFeet;
import frc.robot.commands.EjectBall;
import frc.robot.commands.GrabBall;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.ClimbAngle;
import frc.robot.subsystems.ClimbHook;
import frc.robot.subsystems.ClimbLift;
import frc.robot.commands.Hook_Idle;
import frc.robot.commands.Lift_Idle;
import frc.robot.subsystems.Conveyor;
import frc.robot.commands.SetIntakeDeploy;
import frc.robot.commands.ShootBall_HighPower;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Intake;


public class RobotContainer {

  public final EnhancedJoystick leftJoystick = new EnhancedJoystick(Constants.Driver.leftJoystickPort);
  public final EnhancedJoystick rightJoystick = new EnhancedJoystick(Constants.Driver.rightJoystickPort);

  public final JoystickButton shootBallButton = new JoystickButton(leftJoystick, 1);
  public final JoystickButton grabBallButton = new JoystickButton(leftJoystick, 3);
  public final JoystickButton ejectBallButton = new JoystickButton(leftJoystick, 5);

  public final JoystickButton climbStartButton = new JoystickButton(rightJoystick, 6);
  public final JoystickButton climbFirstBarButton = new JoystickButton(rightJoystick, 7);
  public final JoystickButton climbAdvanceButton = new JoystickButton(rightJoystick, 8);
  public final JoystickButton climbResetButton = new JoystickButton(rightJoystick, 9);
  public final JoystickButton climbCancelButton = new JoystickButton(rightJoystick, 11);
  
  public final CANSparkMax leftDrive0 = new CANSparkMax(Constants.DriveBase.leftSpark0ID, MotorType.kBrushless);
  public final CANSparkMax leftDrive1 = new CANSparkMax(Constants.DriveBase.leftSpark1ID, MotorType.kBrushless);
  public final CANSparkMax rightDrive0 = new CANSparkMax(Constants.DriveBase.rightSpark0ID, MotorType.kBrushless);
  public final CANSparkMax rightDrive1 = new CANSparkMax(Constants.DriveBase.rightSpark1ID, MotorType.kBrushless);
  
  public final Solenoid conveyorPiston = new Solenoid(Constants.Misc.pcmID, PneumaticsModuleType.CTREPCM, Constants.Conveyor.pistonPCMChannel);
  public final WPI_VictorSPX conveyorMotor = new WPI_VictorSPX(Constants.Conveyor.motorID);

  public final Solenoid intakePiston = new Solenoid(Constants.Misc.pcmID, PneumaticsModuleType.CTREPCM, Constants.Intake.pistonPCMChannel);
  public final WPI_VictorSPX intakeMotor = new WPI_VictorSPX(Constants.Intake.intakeMotorID);

  public final UsbCamera cam0 = CameraServer.startAutomaticCapture("Rear Camera", 0);
  public final UsbCamera cam1 = CameraServer.startAutomaticCapture("Front Camera", 1);

  public final CANSparkMax leftLifter = new CANSparkMax(Constants.Climb.lift.leftMotorID, MotorType.kBrushless);
  public final CANSparkMax rightLifter = new CANSparkMax(Constants.Climb.lift.rightMotorID, MotorType.kBrushless);
  public final DigitalInput leftArmFullyDownSwitch = new DigitalInput(Constants.Climb.lift.leftSwitchPort);
  public final DigitalInput rightArmFullyDownSwitch = new DigitalInput(Constants.Climb.lift.rightSwitchPort);

  public final WPI_TalonSRX leftAngleMotor = new WPI_TalonSRX(Constants.Climb.angle.leftMotorID);
  public final WPI_TalonSRX rightAngleMotor = new WPI_TalonSRX(Constants.Climb.angle.rightMotorID);
  public final Solenoid hookPiston = new Solenoid(Constants.Misc.pcmID, PneumaticsModuleType.CTREPCM, Constants.Climb.hook.hookPCMChannel);
  public final DigitalInput leftArmAngleAtZeroSwitch = new DigitalInput(Constants.Climb.angle.leftSwitchPort);
  public final DigitalInput rightArmAngleAtZeroSwitch = new DigitalInput(Constants.Climb.angle.rightSwitchPort);

  public final DriveBase driveBase = new DriveBase(leftDrive0, leftDrive1, rightDrive0, rightDrive1);
  public final Conveyor conveyor = new Conveyor(conveyorMotor, conveyorPiston);
  public final Intake intake = new Intake(intakeMotor, intakePiston);
  public final ClimbLift climbLift = new ClimbLift(leftLifter, rightLifter, leftArmFullyDownSwitch, rightArmFullyDownSwitch);
  public final ClimbAngle climbAngle = new ClimbAngle(leftAngleMotor, rightAngleMotor, leftArmAngleAtZeroSwitch, rightArmAngleAtZeroSwitch);
  public final ClimbHook climbHook = new ClimbHook(hookPiston);

  public final SequentialCommandGroup liftIntakeDelayed = new SequentialCommandGroup(new WaitCommand(1), new SetIntakeDeploy(intake, false));
  final SendableChooser<Command> autonomousChooser = new SendableChooser<>();

  public RobotContainer() {

    // buttons
    configureButtonBindings();
    
    // configure devices
    leftDrive0.setInverted(true);
    leftDrive1.setInverted(true);
    driveBase.setEncoderConversionFactor(Constants.DriveBase.encoderConversionFactor);  // inches, inches/sec
    conveyorMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.setInverted(true);
    cam0.setFPS(15);
    cam0.setResolution(150, 113);
    cam1.setFPS(15);
    cam1.setResolution(150, 113);
    rightLifter.setInverted(true);
    leftLifter.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightLifter.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftAngleMotor.setInverted(true);
    leftAngleMotor.configFactoryDefault();
    rightAngleMotor.configFactoryDefault();
    leftAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rightAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    leftAngleMotor.configSelectedFeedbackCoefficient(Constants.Climb.angle.encoderConversionFactor);
    rightAngleMotor.configSelectedFeedbackCoefficient(Constants.Climb.angle.encoderConversionFactor);
    leftAngleMotor.setNeutralMode(NeutralMode.Coast);
    rightAngleMotor.setNeutralMode(NeutralMode.Coast);
    conveyorMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.setInverted(true); 
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    
    // default commands
    driveBase.setDefaultCommand(new TankDrive(driveBase, () -> leftJoystick.getY(), () -> rightJoystick.getY()));
    climbLift.setDefaultCommand(new Lift_Idle(climbLift));
    climbAngle.setDefaultCommand(new Angle_Idle(climbAngle));
    climbHook.setDefaultCommand(new Hook_Idle(climbHook));

    // set up auto chooser
    configureChooser();
    SmartDashboard.putData(autonomousChooser);
  }

  public void configureChooser() {
    autonomousChooser.setDefaultOption("Do Nothing Autonomous", new InstantCommand());
    autonomousChooser.addOption("Drive forward", new Auto_DriveThreeFeet(driveBase));
    autonomousChooser.addOption("Grab ball and shoot", new AutoGrabShootBall(driveBase, conveyor, intake));
    autonomousChooser.addOption("At goal: shoot, grab, shoot", new AutoAtGoal(driveBase, conveyor, intake));
  }

  private void configureButtonBindings() {
    
    climbStartButton.whenPressed(new Climb_LiftArm(climbAngle, climbHook, climbLift));
    climbFirstBarButton.whenPressed(new Climb_FirstBar(climbAngle, climbHook, climbLift));
    climbAdvanceButton.whenPressed(new Climb_NextBar(climbAngle, climbHook, climbLift));
    climbResetButton.whenPressed(new Climb_CalibrateArmNoSwitch(climbAngle, climbLift));
    climbCancelButton.whenPressed(new ParallelCommandGroup(
      new Angle_Idle(climbAngle),
      new Lift_Idle(climbLift),
      new Hook_Idle(climbHook)
    ));

    grabBallButton.whileHeld(new GrabBall(intake, conveyor));
    grabBallButton.whenReleased(liftIntakeDelayed);

    ejectBallButton.whenHeld(new EjectBall(intake, conveyor));

    shootBallButton.whileHeld(new ShootBall_HighPower(conveyor));

  }

  public Command getAutonomousCommand() {
    return new ParallelCommandGroup(autonomousChooser.getSelected(), new Climb_CalibrateArmNoSwitch(climbAngle, climbLift));
  }
}
