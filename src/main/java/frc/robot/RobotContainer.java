package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.misc2022.EnhancedJoystick;
import frc.misc2022.Gamepad;
import frc.robot.Constants;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


public class RobotContainer {

  public final EnhancedJoystick leftJoystick = new EnhancedJoystick(Constants.Driver.leftJoystickPort);
  public final EnhancedJoystick rightJoystick = new EnhancedJoystick(Constants.Driver.rightJoystickPort);
  public final Gamepad manipulator = new Gamepad(Constants.Driver.manipulatorPort);

  public final DriveBase driveBase;
  public final Conveyor conveyor;

  public final CANSparkMax leftDrive0 = new CANSparkMax(Constants.DriveBase.leftSpark0ID, MotorType.kBrushless);
  public final CANSparkMax leftDrive1 = new CANSparkMax(Constants.DriveBase.leftSpark1ID, MotorType.kBrushless);
  public final CANSparkMax rightDrive0 = new CANSparkMax(Constants.DriveBase.rightSpark0ID, MotorType.kBrushless);
  public final CANSparkMax rightDrive1 = new CANSparkMax(Constants.DriveBase.rightSpark1ID, MotorType.kBrushless);

  public final Solenoid gatePiston = new Solenoid(Constants.Misc.pcmID, PneumaticsModuleType.CTREPCM, Constants.Conveyor.gatePCMChannel);
  public final WPI_VictorSPX conveyorMotor = new WPI_VictorSPX(Constants.Conveyor.motorID);
  

  public RobotContainer() {
    configureButtonBindings();

    driveBase = new DriveBase(leftDrive0, leftDrive1, rightDrive0, rightDrive1);
    // getDistance returns inches, getRate returns inches/second
    driveBase.setDistancePerPulse((1.0 / Constants.DriveBase.gearboxReductionFactor) * Constants.DriveBase.wheelSize * Math.PI);

    driveBase.setDefaultCommand(new TankDrive(driveBase, () -> leftJoystick.getY(), () -> rightJoystick.getY()));

    conveyor = new Conveyor(conveyorMotor, gatePiston);

  }

  private void configureButtonBindings() {}


  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
