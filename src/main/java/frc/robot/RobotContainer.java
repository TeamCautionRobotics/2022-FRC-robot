package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.misc2022.EnhancedJoystick;
import frc.misc2022.Gamepad;
import frc.robot.commands.Climb_Testing;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.ClimbAngle;
import frc.robot.subsystems.ClimbHook;
import frc.robot.subsystems.ClimbLift;
import frc.robot.subsystems.DriveBase;


public class RobotContainer {

  public final EnhancedJoystick leftJoystick = new EnhancedJoystick(Constants.Driver.leftJoystickPort);
  public final EnhancedJoystick rightJoystick = new EnhancedJoystick(Constants.Driver.rightJoystickPort);
  public final Gamepad manipulator = new Gamepad(Constants.Driver.manipulatorPort);

  public final DriveBase driveBase;
  public final ClimbLift climbLift;
  public final ClimbAngle climbAngle;
  public final ClimbHook climbHook;

  public final CANSparkMax leftDrive0 = new CANSparkMax(Constants.DriveBase.leftSpark0ID, MotorType.kBrushless);
  public final CANSparkMax leftDrive1 = new CANSparkMax(Constants.DriveBase.leftSpark1ID, MotorType.kBrushless);
  public final CANSparkMax rightDrive0 = new CANSparkMax(Constants.DriveBase.rightSpark0ID, MotorType.kBrushless);
  public final CANSparkMax rightDrive1 = new CANSparkMax(Constants.DriveBase.rightSpark1ID, MotorType.kBrushless);

  public final CANSparkMax leftLifter = new CANSparkMax(Constants.Climb.lift.leftMotorID, MotorType.kBrushless);
  public final CANSparkMax rightLifter = new CANSparkMax(Constants.Climb.lift.rightMotorID, MotorType.kBrushless);
  public final DigitalInput leftArmFullyDownSwitch = new DigitalInput(Constants.Climb.lift.leftArmFullyDownSwitchPort);
  public final DigitalInput rightArmFullyDownSwitch = new DigitalInput(Constants.Climb.lift.rightArmFullyDownSwtichPort);

  public final WPI_TalonSRX leftAngleMotor = new WPI_TalonSRX(Constants.Climb.angler.leftID);
  public final WPI_TalonSRX rightAngleMotor = new WPI_TalonSRX(Constants.Climb.angler.rightID);
  public final Solenoid hookPiston = new Solenoid(Constants.Misc.pcmID, PneumaticsModuleType.CTREPCM, Constants.Climb.hook.hookPCMChannel);
  public final DigitalInput leftArmAngleAtZeroSwitch = new DigitalInput(Constants.Climb.angler.leftArmAtZeroSwitchPort);
  public final DigitalInput rightArmAngleAtZeroSwitch = new DigitalInput(Constants.Climb.angler.rightArmAtZeroSwitchPort);


  public RobotContainer() {
    configureButtonBindings();

    driveBase = new DriveBase(leftDrive0, leftDrive1, rightDrive0, rightDrive1);
    climbLift = new ClimbLift(leftLifter, rightLifter, leftArmFullyDownSwitch, rightArmFullyDownSwitch);
    climbAngle = new ClimbAngle(leftAngleMotor, rightAngleMotor, leftArmAngleAtZeroSwitch, rightArmAngleAtZeroSwitch);
    climbHook = new ClimbHook(hookPiston);

    // configure things
    // getDistance returns inches, getRate returns inches/second
    driveBase.setDistancePerPulse((1.0 / Constants.DriveBase.gearboxReductionFactor) * Constants.DriveBase.wheelSize * Math.PI);

    // set the idle mode of the lift motors to braking
    leftLifter.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightLifter.setIdleMode(CANSparkMax.IdleMode.kBrake);

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
    leftAngleMotor.setNeutralMode(NeutralMode.Brake);
    rightAngleMotor.setNeutralMode(NeutralMode.Brake);


    // set default commands
    driveBase.setDefaultCommand(new TankDrive(driveBase, () -> leftJoystick.getY(), () -> rightJoystick.getY()));
    climbLift.setDefaultCommand(new Climb_Testing(climbAngle, climbHook, climbLift));

  }

  private void configureButtonBindings() {}


  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
