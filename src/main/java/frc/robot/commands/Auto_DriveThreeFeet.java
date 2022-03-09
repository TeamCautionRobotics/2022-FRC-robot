package frc.robot.commands;

import frc.robot.subsystems.DriveBase;

public class Auto_DriveThreeFeet extends AutoDriveDistance {

  /**
   * This command will automatically drive the bot forward no less than 3 feet
   * 
   * @param driveBase
   */
  public Auto_DriveThreeFeet(DriveBase driveBase) {
    super(driveBase, 36);
  }
}
