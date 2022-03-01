package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ClimbAngle;
import frc.robot.subsystems.ClimbHook;
import frc.robot.subsystems.ClimbLift;

import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Climb_LiftAndLatch extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbAngle angleSubsystem;
  private final ClimbHook hookSubsystem;
  private final ClimbLift liftSubsystem; 

  private boolean commandDone = false;

  private int climbStep = 1;


  /**
   * creates a new Climb_LiftAndLatch commnad.
   * pulls down and hooks onto the first bar, lifts and latches using the static hooks.
   * @param angleSubsystem
   * @param hookSubsystem
   * @param liftSubsystem
   */
  public Climb_LiftAndLatch(ClimbAngle angleSubsystem, ClimbHook hookSubsystem, ClimbLift liftSubsystem) {
    
    addRequirements(angleSubsystem, hookSubsystem, liftSubsystem);

    this.angleSubsystem = angleSubsystem;
    this.hookSubsystem = hookSubsystem;
    this.liftSubsystem = liftSubsystem;

  }

  @Override
  public void initialize() {

    // check if arm is raised
    if ((liftSubsystem.getLeftLiftPosition() < Constants.Climb.lift.fullyRaisedThreshold) && 
        (liftSubsystem.getRightLiftPosition() < Constants.Climb.lift.fullyRaisedThreshold) &&
        (angleSubsystem.getLeftAngleEncoderDistance() < Constants.Climb.angler.raisedThreshold) &&
        (angleSubsystem.getRightAngleEncoderDistance() < Constants.Climb.angler.raisedThreshold)) {

          System.out.println("WARNING: Tried activating climb command when arm was not raised!");
          commandDone = true;

        }

        // start on step 1
        climbStep = 1;

  }

  @Override
  public void execute() {

    // SAFE MODE
    if (climbStep == 0) {
      
      // force the hooks out
      hookSubsystem.setHook(true);

      // force cut power to angle motors
      angleSubsystem.enableAnglePid(false);
      angleSubsystem.stop();

      // slowly lower the lift until it is fully lowered
      if ((liftSubsystem.getLeftLiftPosition() + liftSubsystem.getRightLiftPosition()) / 2 < Constants.Climb.lift.fullyRaisedThreshold) {
        liftSubsystem.setReference(Constants.Climb.lift.safeModeLoweringVelocity, ControlType.kVelocity);
      } else {
        liftSubsystem.stop();
        commandDone = true;
      }



    // the first step of climbing 
    } else if (climbStep == 1) {

    } else {
      // engage fallback state
    }

    // check if motor current has exceeded threshold, if so, activate safe mode

    if (liftSubsystem.getMotorCurrent(0) > Constants.Climb.lift.maxCurrentThreshold ||
        liftSubsystem.getMotorCurrent(1) > Constants.Climb.lift.maxCurrentThreshold) {

          System.out.println("ERROR: LIFT MOTORS HAVE EXCEEDED THE CURRENT LIMIT! ACTIVATING SAFE MODE");
          climbStep = 0;

        }


  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return commandDone;
  }
}