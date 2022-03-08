package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.misc2022.EnhancedJoystick;
import frc.misc2022.Gamepad;
import frc.robot.commands.Angle_Idle;
import frc.robot.commands.Climb_CalibrateArm;
import frc.robot.commands.Climb_FirstBar;
import frc.robot.commands.Climb_LiftArm;
import frc.robot.commands.Climb_Testing;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.ClimbAngle;
import frc.robot.subsystems.ClimbHook;
import frc.robot.subsystems.ClimbLift;
import frc.robot.commands.GrabBall;
import frc.robot.commands.Hook_Idle;
import frc.robot.commands.Lift_Idle;
import frc.robot.commands.RunConveyorMotor;
import frc.robot.commands.ToggleConveyorGate;
import frc.robot.subsystems.Conveyor;
import frc.robot.commands.RunIntakeMotor;
import frc.robot.commands.ShootBall;
import frc.robot.commands.ToggleIntakeDeploy;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Intake;


public class RobotContainer {

  public final EnhancedJoystick leftJoystick = new EnhancedJoystick(Constants.Driver.leftJoystickPort);
  public final EnhancedJoystick rightJoystick = new EnhancedJoystick(Constants.Driver.rightJoystickPort);
  public final Gamepad manipulator = new Gamepad(Constants.Driver.manipulatorPort);

  public final JoystickButton shootBallButton = new JoystickButton(leftJoystick, 1);
  public final JoystickButton grabBallButton = new JoystickButton(leftJoystick, 3);

  public final JoystickButton intakeDeployButton = new JoystickButton(leftJoystick, 6);
  public final JoystickButton intakeMotorButton = new JoystickButton(leftJoystick, 7);  
  public final JoystickButton conveyorGateButton = new JoystickButton(rightJoystick, 11);
  public final JoystickButton conveyorMotorButton = new JoystickButton(rightJoystick, 10);

  public final JoystickButton calibrateArmButton = new JoystickButton(rightJoystick, 6);
  public final JoystickButton liftArmButton = new JoystickButton(rightJoystick, 7);
  public final JoystickButton firstBarButton = new JoystickButton(rightJoystick, 8);

  public final CANSparkMax leftDrive0 = new CANSparkMax(Constants.DriveBase.leftSpark0ID, MotorType.kBrushless);
  public final CANSparkMax leftDrive1 = new CANSparkMax(Constants.DriveBase.leftSpark1ID, MotorType.kBrushless);
  public final CANSparkMax rightDrive0 = new CANSparkMax(Constants.DriveBase.rightSpark0ID, MotorType.kBrushless);
  public final CANSparkMax rightDrive1 = new CANSparkMax(Constants.DriveBase.rightSpark1ID, MotorType.kBrushless);
  
  public final Solenoid gatePiston = new Solenoid(Constants.Misc.pcmID, PneumaticsModuleType.CTREPCM, Constants.Conveyor.gatePCMChannel);
  public final WPI_VictorSPX conveyorMotor = new WPI_VictorSPX(Constants.Conveyor.motorID);

  public final WPI_VictorSPX intakeMotor = new WPI_VictorSPX(Constants.Intake.intakeMotorID);
  public final Solenoid intakeDeploy = new Solenoid(Constants.Misc.pcmID, PneumaticsModuleType.CTREPCM, Constants.Intake.pistonPCMChannel);

  public final CANSparkMax leftLifter = new CANSparkMax(Constants.Climb.lift.leftMotorID, MotorType.kBrushless);
  public final CANSparkMax rightLifter = new CANSparkMax(Constants.Climb.lift.rightMotorID, MotorType.kBrushless);
  public final DigitalInput leftArmFullyDownSwitch = new DigitalInput(Constants.Climb.lift.leftArmFullyDownSwitchPort);
  public final DigitalInput rightArmFullyDownSwitch = new DigitalInput(Constants.Climb.lift.rightArmFullyDownSwtichPort);

  public final WPI_TalonSRX leftAngleMotor = new WPI_TalonSRX(Constants.Climb.angler.leftID);
  public final WPI_TalonSRX rightAngleMotor = new WPI_TalonSRX(Constants.Climb.angler.rightID);
  public final Solenoid hookPiston = new Solenoid(Constants.Misc.pcmID, PneumaticsModuleType.CTREPCM, Constants.Climb.hook.hookPCMChannel);
  public final DigitalInput leftArmAngleAtZeroSwitch = new DigitalInput(Constants.Climb.angler.leftArmAtZeroSwitchPort);
  public final DigitalInput rightArmAngleAtZeroSwitch = new DigitalInput(Constants.Climb.angler.rightArmAtZeroSwitchPort);

  public final DriveBase driveBase = new DriveBase(leftDrive0, leftDrive1, rightDrive0, rightDrive1);
  public final Conveyor conveyor = new Conveyor(conveyorMotor, gatePiston);
  public final Intake intake = new Intake(intakeMotor, intakeDeploy);
  public final ClimbLift climbLift = new ClimbLift(leftLifter, rightLifter, leftArmFullyDownSwitch, rightArmFullyDownSwitch);
  public final ClimbAngle climbAngle = new ClimbAngle(leftAngleMotor, rightAngleMotor, leftArmAngleAtZeroSwitch, rightArmAngleAtZeroSwitch);
  public final ClimbHook climbHook = new ClimbHook(hookPiston);


  public RobotContainer() {
    configureButtonBindings();


    // configure things
    // getDistance returns inches, getRate returns inches/second
    driveBase.setDistancePerPulse((1.0 / Constants.DriveBase.gearboxReductionFactor) * Constants.DriveBase.wheelSize * Math.PI);

    rightLifter.setInverted(true);

    // set the idle mode of the lift motors to braking
    leftLifter.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightLifter.setIdleMode(CANSparkMax.IdleMode.kBrake);

    leftAngleMotor.setInverted(true);

    // reset angle motors to defaults
    leftAngleMotor.configFactoryDefault();
    rightAngleMotor.configFactoryDefault();
    // set the feedback devices to quad encoders
    leftAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rightAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    // set the angle encoder coefficient 
    leftAngleMotor.configSelectedFeedbackCoefficient(Constants.Climb.angler.encoderDistancePerPulse);
    rightAngleMotor.configSelectedFeedbackCoefficient(Constants.Climb.angler.encoderDistancePerPulse);
    // set the idle mode to braking
    leftAngleMotor.setNeutralMode(NeutralMode.Coast);
    rightAngleMotor.setNeutralMode(NeutralMode.Coast);


    // set default commands
    CameraServer.startAutomaticCapture("Rear Camera", 0);
    CameraServer.startAutomaticCapture("Front Camera", 1);

    // config
    conveyorMotor.setNeutralMode(NeutralMode.Brake);  // brake the conveyor when stopped

    intakeMotor.setInverted(true);  // invert the intake motor
    intakeMotor.setNeutralMode(NeutralMode.Brake);  // brake the intake when stopped

    driveBase.setDistancePerPulse((1.0 / Constants.DriveBase.gearboxReductionFactor) * Constants.DriveBase.wheelSize * Math.PI);  // getDistance returns inches, getRate returns inches/second
    
    // default commands
    driveBase.setDefaultCommand(new TankDrive(driveBase, () -> leftJoystick.getY(), () -> rightJoystick.getY()));
    
    climbLift.setDefaultCommand(new Lift_Idle(climbLift));
    climbAngle.setDefaultCommand(new Angle_Idle(climbAngle));
    climbHook.setDefaultCommand(new Hook_Idle(climbHook));

  }

  private void configureButtonBindings() {

    conveyorGateButton.whenPressed(new ToggleConveyorGate(conveyor));
    conveyorMotorButton.whenHeld(new RunConveyorMotor(conveyor));
    
    intakeDeployButton.whenPressed(new ToggleIntakeDeploy(intake));
    intakeMotorButton.whileHeld(new RunIntakeMotor(intake));

    calibrateArmButton.whenPressed(new Climb_CalibrateArm(climbAngle, climbLift));
    liftArmButton.whenPressed(new Climb_LiftArm(climbAngle, climbHook, climbLift));
    firstBarButton.whenPressed(new Climb_FirstBar(climbAngle, climbHook, climbLift));
    
    grabBallButton.whileHeld(new GrabBall(intake, conveyor));
    shootBallButton.whileHeld(new ShootBall(conveyor));

  }


  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
