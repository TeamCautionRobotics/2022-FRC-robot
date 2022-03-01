package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.misc2022.EnhancedJoystick;
import frc.misc2022.Gamepad;
import frc.robot.commands.SetIntakeMotor;
import frc.robot.commands.TankDrive;
import frc.robot.commands.ToggleIntakeDeploy;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Intake;


public class RobotContainer {

  public final EnhancedJoystick leftJoystick = new EnhancedJoystick(Constants.Driver.leftJoystickPort);
  public final EnhancedJoystick rightJoystick = new EnhancedJoystick(Constants.Driver.rightJoystickPort);
  public final Gamepad manipulator = new Gamepad(Constants.Driver.manipulatorPort);

  public final JoystickButton intakeDeployButton = new JoystickButton(leftJoystick, 2);
  public final JoystickButton intakeMotorButton = new JoystickButton(leftJoystick, 3);

  public final DriveBase driveBase;
  public final Intake intake;

  public final CANSparkMax leftDrive0 = new CANSparkMax(Constants.DriveBase.leftSpark0ID, MotorType.kBrushless);
  public final CANSparkMax leftDrive1 = new CANSparkMax(Constants.DriveBase.leftSpark1ID, MotorType.kBrushless);
  public final CANSparkMax rightDrive0 = new CANSparkMax(Constants.DriveBase.rightSpark0ID, MotorType.kBrushless);
  public final CANSparkMax rightDrive1 = new CANSparkMax(Constants.DriveBase.rightSpark1ID, MotorType.kBrushless);

  public final WPI_VictorSPX intakeMotor = new WPI_VictorSPX(Constants.Intake.intakeMotorID);
  public final Solenoid intakeDeploy = new Solenoid(Constants.Misc.pcmID, PneumaticsModuleType.CTREPCM, Constants.Intake.pistonPCMChannel);

  public RobotContainer() {
    configureButtonBindings();

    driveBase = new DriveBase(leftDrive0, leftDrive1, rightDrive0, rightDrive1);
    intake = new Intake(intakeMotor, intakeDeploy);


    // getDistance returns inches, getRate returns inches/second
    driveBase.setDistancePerPulse((1.0 / Constants.DriveBase.gearboxReductionFactor) * Constants.DriveBase.wheelSize * Math.PI);

    driveBase.setDefaultCommand(new TankDrive(driveBase, () -> leftJoystick.getY(), () -> rightJoystick.getY()));

  }

  private void configureButtonBindings() {

    intakeDeployButton.whenPressed(new ToggleIntakeDeploy(intake));
    
    intakeMotorButton.whileHeld(new SetIntakeMotor(intake, 1.0));
    intakeMotorButton.whenReleased(new SetIntakeMotor(intake, 0.0));

  }


  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
