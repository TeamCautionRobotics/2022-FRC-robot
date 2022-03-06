package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ClimbAngle;
import frc.robot.subsystems.ClimbHook;
import frc.robot.subsystems.ClimbLift;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Climb_LiftAndHook extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbAngle angleSubsystem;
  private final ClimbHook hookSubsystem;
  private final ClimbLift liftSubsystem; 

  private Timer timer0 = new Timer();
  private boolean stateVar0 = false;

  private boolean commandDone = false;
  private int climbStep = 1;

  /**
   * pull down and latch. this assumes arms are calibrated and raised above the first bar
   * @param angleSubsystem
   * @param hookSubsystem
   * @param liftSubsystem
   */
  public Climb_LiftAndHook(ClimbAngle angleSubsystem, ClimbHook hookSubsystem, ClimbLift liftSubsystem) {
    
    addRequirements(angleSubsystem, hookSubsystem, liftSubsystem);

    this.angleSubsystem = angleSubsystem;
    this.hookSubsystem = hookSubsystem;
    this.liftSubsystem = liftSubsystem;

  }

  @Override
  public void initialize() {

    if (  // check if the arm has been raised
      (liftSubsystem.getLeftLiftPosition() < 19.8) ||
      (liftSubsystem.getRightLiftPosition() < 19.8) ||
      (angleSubsystem.getLeftEncoderDistance() < 59.8) ||
      (angleSubsystem.getRightEncoderDistance() < 59.8)) {

      commandDone = true;
      climbStep = 1;  // jump to nothing loop
      System.out.println("WARNING: Tried to activate climb when arm was not raised! Aborting...");

    } else {

      // initialize things
      timer0.reset();
      timer0.stop();
      stateVar0 = false;

      // start on 10
      climbStep = 10;

    }

  }

  @Override
  public void execute() {

    switch(climbStep) {

      case 0:  // SAFE MODE

        System.out.println("WARNING! LIFT MOTORS EXCEEDED CURRENT THRESHOLD! ACTIVATING SAFE MODE!");

        // extend the hooks
        hookSubsystem.set(true);

        // Disable the PIDs
        angleSubsystem.enablePID(false);
        liftSubsystem.enablePID(false);

        // force idle mode of lift to braking
        liftSubsystem.setIdleMode(IdleMode.kBrake);

        // kill power to motors
        angleSubsystem.setPower(0);
        liftSubsystem.setPower(0);
        angleSubsystem.stop();
        liftSubsystem.stop();

        // end
        climbStep = 1;
        commandDone = true;  

      case 1:  // do nothing
        ;

      case 10:  // start pulling down

        if (!stateVar0) {  // retract hooks (only if not retracted)
          hookSubsystem.set(false); 
          stateVar0 = true;
        }

        liftSubsystem.setPosition(0.0);  // come down to zero position

        if (  // once we hit 15 inches out, advance to the next step
          (liftSubsystem.getRightLiftPosition() < 15.0) ||
          (liftSubsystem.getRightLiftPosition() < 15.0)) {
            climbStep = 11; 
        }

      case 11:  // cut power to the angle motors

        angleSubsystem.enablePID(false);
        angleSubsystem.setPower(0);

        if (  // once we're on target, advance to the next step, reset and start the timer
        (liftSubsystem.getRightLiftPosition() < 0.2) ||
        (liftSubsystem.getRightLiftPosition() < 0.2)) {

          stateVar0 = false;
          timer0.reset();
          timer0.start();
        
          climbStep = 12; 
      }
      
      case 12:  // engage the hooks

        if (!stateVar0) {  // only if not engaged
          hookSubsystem.set(true);
          stateVar0 = true;
        }

        if (timer0.get() > 0.5) {  // if we've gone more than half a second, advance to the next step

          timer0.stop();
          timer0.reset();

          climbStep = 13;

        }

      case 13:  // drop two inches to make sure we're resting on hooks and finish
        
        liftSubsystem.setPosition(2);
        commandDone = true;

    }

    if (  // safe mode trigger
      (liftSubsystem.getLeftMotorCurrent() > Constants.Climb.lift.maxCurrentThreshold) ||
      (liftSubsystem.getRightMotorCurrent() > Constants.Climb.lift.maxCurrentThreshold)
      ) {

        climbStep = 0;  // jump to step 0 (safe mode)

    }

  }

  @Override
  public boolean isFinished() {
    return commandDone;
  }
}
