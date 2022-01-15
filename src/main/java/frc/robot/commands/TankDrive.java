package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class TankDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final DriveBase driveBase;

  private final DoubleSupplier leftPowerSupplier;
  private final DoubleSupplier rightPowerSupplier;

  public TankDrive(DriveBase subsystem, DoubleSupplier leftPowerSupplier, DoubleSupplier rightPowerSupplier) {
    this.driveBase = subsystem;

    this.leftPowerSupplier = leftPowerSupplier;
    this.rightPowerSupplier = rightPowerSupplier;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    driveBase.drive(leftPowerSupplier.getAsDouble(), rightPowerSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    driveBase.drive(0,0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
