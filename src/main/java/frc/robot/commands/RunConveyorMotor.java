package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class RunConveyorMotor extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Conveyor conveyorSubsystem;

  public RunConveyorMotor(Conveyor conveyor) {
    this.conveyorSubsystem = conveyor;

    addRequirements(conveyor);
  }

  @Override
  public void execute() {
    conveyorSubsystem.runMotor(1.0);
  }

  @Override
  public void end(boolean interrupted) {
    conveyorSubsystem.runMotor(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
