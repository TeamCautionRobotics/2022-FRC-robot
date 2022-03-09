package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Intake;

public class AutoGrabShootBall extends SequentialCommandGroup {
  public AutoGrabShootBall(DriveBase driveBase, Conveyor conveyor, Intake intake) {
    addCommands(
      // Prepare for reaping the ball
      new SetIntakeDeploy(intake, true),

      // drive backwards while reaping. Stop reaping slightly after driving finishes
      new ParallelDeadlineGroup(
        // run GrabBall command for as long as drive followed by wait takes
        new SequentialCommandGroup(
          new AutoDriveDistance(driveBase, -41),  // I think this distance is correct.
          new WaitCommand(0.5)  // wait a bit for the ball to be succ and the robot to stop.
        ),
        new GrabBall(intake, conveyor)
      ),
      
      // drive forwards for sowing ball in the goal
      new AutoDriveDistance(driveBase, 60), // TODO: check distance from ball to goal
          // also, TODO: should this have a timer as a fallback completetion
      
      // sow the ball into the goal and switch off after five (5) seconds
      new ParallelDeadlineGroup(
        new WaitCommand(5),
        new ShootBall(conveyor)
      )
    );
  }
}
