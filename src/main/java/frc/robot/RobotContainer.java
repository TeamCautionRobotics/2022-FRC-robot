package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.misc2022.EnhancedJoystick;
import frc.robot.commands.AutoAtGoal;
import frc.robot.commands.AutoGrabShootBall;
import frc.robot.commands.Auto_DriveThreeFeet;
import frc.robot.commands.EjectBall;
import frc.robot.commands.GrabBall;
import frc.robot.commands.TankDrive;
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

  public final DriveBase driveBase = new DriveBase(leftDrive0, leftDrive1, rightDrive0, rightDrive1);
  public final Conveyor conveyor = new Conveyor(conveyorMotor, conveyorPiston);
  public final Intake intake = new Intake(intakeMotor, intakePiston);

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
    intakeMotor.setInverted(true);
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    cam0.setFPS(15);
    cam1.setFPS(15);
    
    // default commands
    driveBase.setDefaultCommand(new TankDrive(driveBase, () -> leftJoystick.getY(), () -> rightJoystick.getY()));

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

    grabBallButton.whileHeld(new GrabBall(intake, conveyor));
    grabBallButton.whenReleased(liftIntakeDelayed);

    ejectBallButton.whenHeld(new EjectBall(intake, conveyor));

    shootBallButton.whileHeld(new ShootBall_HighPower(conveyor));

  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }
}
