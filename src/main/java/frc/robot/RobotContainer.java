package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.misc2022.EnhancedJoystick;
import frc.misc2022.Gamepad;
import frc.robot.Constants;
import frc.robot.commands.AutoGrabShootBall;
import frc.robot.commands.AutoGrabShootBall2;
import frc.robot.commands.Auto_DriveThreeFeet;
import frc.robot.commands.GrabBall;
import frc.robot.commands.RunConveyorMotor;
import frc.robot.commands.TankDrive;
import frc.robot.commands.ToggleConveyorGate;
import frc.robot.subsystems.Conveyor;
import frc.robot.commands.RunIntakeMotor;
import frc.robot.commands.ShootBall;
import frc.robot.commands.ToggleIntakeDeploy;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Intake;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


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

  public final CANSparkMax leftDrive0 = new CANSparkMax(Constants.DriveBase.leftSpark0ID, MotorType.kBrushless);
  public final CANSparkMax leftDrive1 = new CANSparkMax(Constants.DriveBase.leftSpark1ID, MotorType.kBrushless);
  public final CANSparkMax rightDrive0 = new CANSparkMax(Constants.DriveBase.rightSpark0ID, MotorType.kBrushless);
  public final CANSparkMax rightDrive1 = new CANSparkMax(Constants.DriveBase.rightSpark1ID, MotorType.kBrushless);
  
  public final Solenoid gatePiston = new Solenoid(Constants.Misc.pcmID, PneumaticsModuleType.CTREPCM, Constants.Conveyor.gatePCMChannel);
  public final WPI_VictorSPX conveyorMotor = new WPI_VictorSPX(Constants.Conveyor.motorID);

  public final WPI_VictorSPX intakeMotor = new WPI_VictorSPX(Constants.Intake.intakeMotorID);
  public final Solenoid intakeDeploy = new Solenoid(Constants.Misc.pcmID, PneumaticsModuleType.CTREPCM, Constants.Intake.pistonPCMChannel);

  public final DriveBase driveBase = new DriveBase(leftDrive0, leftDrive1, rightDrive0, rightDrive1);
  public final Conveyor conveyor = new Conveyor(conveyorMotor, gatePiston);
  public final Intake intake = new Intake(intakeMotor, intakeDeploy);

  final SendableChooser<Command> autonomousChooser = new SendableChooser<>();

  public RobotContainer() {
    configureButtonBindings();

    CameraServer.startAutomaticCapture("Rear Camera", 0);
    CameraServer.startAutomaticCapture("Front Camera", 1);

    // config
    conveyorMotor.setNeutralMode(NeutralMode.Brake);  // brake the conveyor when stopped

    intakeMotor.setInverted(true);  // invert the intake motor
    intakeMotor.setNeutralMode(NeutralMode.Brake);  // brake the intake when stopped

    driveBase.setDistancePerPulse((1.0 / Constants.DriveBase.gearboxReductionFactor) * Constants.DriveBase.wheelSize * Math.PI);  // getDistance returns inches, getRate returns inches/second
    
    // default commands
    driveBase.setDefaultCommand(new TankDrive(driveBase, () -> leftJoystick.getY(), () -> rightJoystick.getY()));

    autonomousChooser.setDefaultOption("Do Nothing Autonomous", new InstantCommand());
    autonomousChooser.addOption("Drive forward", new Auto_DriveThreeFeet(driveBase));
    autonomousChooser.addOption("Grab ball and shoot", new AutoGrabShootBall(driveBase, conveyor, intake));
    autonomousChooser.addOption("Grab ball and shoot two: The sequel", new AutoGrabShootBall2(driveBase, conveyor, intake));
    SmartDashboard.putData(autonomousChooser);
  }

  private void configureButtonBindings() {

    conveyorGateButton.whenPressed(new ToggleConveyorGate(conveyor));
    conveyorMotorButton.whenHeld(new RunConveyorMotor(conveyor));
    
    intakeDeployButton.whenPressed(new ToggleIntakeDeploy(intake));
    intakeMotorButton.whileHeld(new RunIntakeMotor(intake));

    grabBallButton.whileHeld(new GrabBall(intake, conveyor));
    shootBallButton.whileHeld(new ShootBall(conveyor));

  }


  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }
}
