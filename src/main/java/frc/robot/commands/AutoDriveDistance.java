package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class AutoDriveDistance extends CommandBase {

  private final DriveBase driveBase;
  private final double driveDistance;
  private final double drivePower;
  private boolean commandDone = false;

  /**
   * Drive the given distance at 50% power, then stop
   * 
   * @param driveBase
   * @param distance  distance to drive in inches, can be negative.
   */
  public AutoDriveDistance(DriveBase driveBase, double distance) {
    this.driveBase = driveBase;
    driveDistance = distance;
    drivePower = distance > 0 ? -0.5 : 0.5;
    addRequirements(driveBase);
  }

  @Override
  public void initialize() {
    commandDone = false; // wpilib bug workaround
    driveBase.resetEncoders();
    driveBase.setIdleMode(IdleMode.kBrake); // set the idle mode to braking
    driveBase.drive(drivePower); // half power
  }

  @Override
  public void execute() {
    if ((driveDistance > 0 && driveBase.getDistance() > driveDistance)
        || (!(driveDistance > 0) && driveBase.getDistance() < driveDistance)) { // if we've passed 36 inches
      driveBase.drive(0); // stop driving
      commandDone = true; // exit the command
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveBase.setIdleMode(IdleMode.kCoast); // set the idle mode back to coasting
  }

  @Override
  public boolean isFinished() {
    return commandDone;
  }
}
