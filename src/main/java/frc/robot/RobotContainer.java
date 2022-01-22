package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveBase;


public class RobotContainer {

  public final DriveBase driveBase;

  public final CANSparkMax leftDrive0 = new CANSparkMax(Constants.DriveBase.leftSpark0ID, MotorType.kBrushless);
  public final CANSparkMax leftDrive1 = new CANSparkMax(Constants.DriveBase.leftSpark1ID, MotorType.kBrushless);
  public final CANSparkMax rightDrive0 = new CANSparkMax(Constants.DriveBase.rightSpark0ID, MotorType.kBrushless);
  public final CANSparkMax rightDrive1 = new CANSparkMax(Constants.DriveBase.rightSpark1ID, MotorType.kBrushless);

  public RobotContainer() {
    configureButtonBindings();

    driveBase = new DriveBase(leftDrive0, leftDrive1, rightDrive0, rightDrive1);
    // getDistance returns inches, getRate returns inches/second
    driveBase.setDistancePerPulse(1.0 / (Constants.DriveBase.gearboxReductionFactor * Constants.DriveBase.wheelSize * Math.PI));

  }

  private void configureButtonBindings() {}


  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
