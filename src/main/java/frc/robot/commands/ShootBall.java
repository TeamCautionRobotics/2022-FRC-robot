package frc.robot.commands;

import frc.robot.subsystems.Conveyor;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootBall extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Conveyor conveyorSubsystem;

  public ShootBall(Conveyor conveyorSubsystem) {
    this.conveyorSubsystem = conveyorSubsystem;

    addRequirements(conveyorSubsystem);
  }

  @Override
  public void initialize() {
    conveyorSubsystem.setGate(false);  // retract gate
  }

  @Override
  public void execute() {
    conveyorSubsystem.runMotor(0.70);  // run conveyor slower
  }

  @Override
  public void end(boolean interrupted) {
    conveyorSubsystem.runMotor(0.0);  // stop the motor
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
