package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
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

  public final CANSparkMax leftLifter = new CANSparkMax(Constants.Climb.lift.leftID, MotorType.kBrushless);
  public final CANSparkMax rightLifter = new CANSparkMax(Constants.Climb.lift.rightID, MotorType.kBrushless);

  public final WPI_VictorSPX leftAngleMotor = new WPI_VictorSPX(Constants.Climb.angler.leftID);
  public final WPI_VictorSPX rightAngleMotor = new WPI_VictorSPX(Constants.Climb.angler.rightID);
  public final Encoder leftAngleEncoder = new Encoder(Constants.Climb.angler.leftEncoderA, Constants.Climb.angler.leftEncoderB);
  public final Encoder rightAngleEncoder = new Encoder(Constants.Climb.angler.rightEncoderA, Constants.Climb.angler.rightEncoderB);
  public final Solenoid hookPiston = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Climb.hook.ID);


  public RobotContainer() {
    configureButtonBindings();

    driveBase = new DriveBase(leftDrive0, leftDrive1, rightDrive0, rightDrive1);
    climbLift = new ClimbLift(leftLifter, rightLifter);
    climbAngle = new ClimbAngle(leftAngleMotor, rightAngleMotor, leftAngleEncoder, rightAngleEncoder);
    climbHook = new ClimbHook(hookPiston);



    // getDistance returns inches, getRate returns inches/second
    driveBase.setDistancePerPulse((1.0 / Constants.DriveBase.gearboxReductionFactor) * Constants.DriveBase.wheelSize * Math.PI);

    driveBase.setDefaultCommand(new TankDrive(driveBase, () -> leftJoystick.getY(), () -> rightJoystick.getY()));
    climbLift.setDefaultCommand(new Climb_Testing(climbAngle, climbHook, climbLift));



  }

  private void configureButtonBindings() {}


  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
