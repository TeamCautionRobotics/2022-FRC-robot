package frc.robot.commands;

import frc.robot.subsystems.ClimbAngle;
import frc.robot.subsystems.ClimbHook;
import frc.robot.subsystems.ClimbLift;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Climb_LiftArm extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbAngle angleSubsystem;
  private final ClimbHook hookSubsystem;
  private final ClimbLift liftSubsystem; 

  private boolean commandDone = false;
  private int climbStep = 0;


  /**
   * @param angleSubsystem
   * @param hookSubsystem
   * @param liftSubsystem
   */
  public Climb_LiftArm(ClimbAngle angleSubsystem, ClimbHook hookSubsystem, ClimbLift liftSubsystem) {
    
    addRequirements(angleSubsystem, hookSubsystem, liftSubsystem);

    this.angleSubsystem = angleSubsystem;
    this.hookSubsystem = hookSubsystem;
    this.liftSubsystem = liftSubsystem;

  }

  @Override
  public void initialize() {

    // retract the static hooks
    hookSubsystem.set(false);

    // start on step 0
    climbStep = 0;

    commandDone = false;

  }

  @Override
  public void execute() {

    switch(climbStep) {

      case 0:  // extend the lift 20 in

      commandDone = false;
        if ((liftSubsystem.getLeftLiftPosition() > 19.8) &&
           (liftSubsystem.getRightLiftPosition() > 19.8)) {

            climbStep = 1;

        } else {
             liftSubsystem.enablePID(true);  // enable pid
             liftSubsystem.setPosition(20.0);  // set setpoint
        }
        break;

      case 1:  // angle the arms 110 deg

        if ((angleSubsystem.getLeftEncoderDistance() > 100) &&
           (angleSubsystem.getRightEncoderDistance() > 100)) {

            climbStep = 2;

        } else {

          angleSubsystem.enablePID(true);  // enable pid
          angleSubsystem.setPosition(110);  // set setpoint

        }
        break;

      case 2:

        commandDone = true;  // we're done
        break;

    }

  }

  @Override
  public void end(boolean interrupted) {

    if (interrupted) {
      System.out.println("command terminated");
    }

  }

  @Override
  public boolean isFinished() {
    return commandDone;
  }
}
