package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class Auto_DriveThreeFeet extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final DriveBase driveBase;
  private boolean commandDone = false;


  /**
   * This command will automatically drive the bot forward no less than 3 feet
   * @param driveBase
   */
  public Auto_DriveThreeFeet(DriveBase driveBase) {

    this.driveBase = driveBase;

    addRequirements(driveBase);
  }

  @Override
  public void initialize() {

    commandDone = false;  // wpilib bug workaround
    driveBase.setIdleMode(IdleMode.kBrake);  // set the idle mode to braking
    driveBase.drive(0.5); //  half power

  }

  @Override
  public void execute() {

    if (driveBase.getDistance() > 36) {  // if we've passed 36 inches

      driveBase.drive(0);  // stop driving
      commandDone = true;  // exit the command

    }

  }

  @Override
  public void end(boolean interrupted) {

    driveBase.setIdleMode(IdleMode.kCoast);  // set the idle mode back to coasting

  }

  @Override
  public boolean isFinished() {
    return commandDone;
  }
}
