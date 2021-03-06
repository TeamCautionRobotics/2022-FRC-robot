package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ClimbAngle;
import frc.robot.subsystems.ClimbHook;
import frc.robot.subsystems.ClimbLift;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

public class Climb_FirstBar extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbAngle angleSubsystem;
  private final ClimbHook hookSubsystem;
  private final ClimbLift liftSubsystem; 

  private boolean commandDone = false;
  private int climbStep = 0;
  private Timer currentLimitTimeout = new Timer();
  private Timer timer0 = new Timer();


  /**
   * @param angleSubsystem
   * @param hookSubsystem
   * @param liftSubsystem
   */
  public Climb_FirstBar(ClimbAngle angleSubsystem, ClimbHook hookSubsystem, ClimbLift liftSubsystem) {
    
    addRequirements(angleSubsystem, hookSubsystem, liftSubsystem);

    this.angleSubsystem = angleSubsystem;
    this.hookSubsystem = hookSubsystem;
    this.liftSubsystem = liftSubsystem;

  }

  @Override
  public void initialize() {

    commandDone = false;  // wpilib bug workaround

    if (!angleSubsystem.getCalibrated() || !liftSubsystem.getCalibrated()) {
      climbStep = 13;
    } else {

      currentLimitTimeout.start();  // start the current timeout timer

      hookSubsystem.set(false);  // retract the static hooks
      angleSubsystem.setNeutralMode(NeutralMode.Coast);  // coast the arm motors
      liftSubsystem.setIdleMode(IdleMode.kBrake);  // brake the lift motors

      climbStep = 10;  // start on step 10

    }

  }

  @Override
  public void execute() {

    switch(climbStep) {

      case 0:  // SAFE MODE

        System.out.println("WARNING: Winch exceeded current limit! Activating safe mode...");

        hookSubsystem.set(true);  // extend the hooks

        // brake all motors
        angleSubsystem.setNeutralMode(NeutralMode.Brake);
        liftSubsystem.setIdleMode(IdleMode.kBrake);

        // cut all power to angle
        angleSubsystem.enablePID(false);
        angleSubsystem.setPower(0);
        angleSubsystem.stop();

        // cut all power to lift
        liftSubsystem.enablePID(false);
        liftSubsystem.setPower(0);
        liftSubsystem.stop();

        commandDone = true;  // we're done
        climbStep = 1;  // do nothing for remainder of command
        break;

      case 1:
        ;  // do nothing
        break;

      case 10:  // pull lift down

        if ((liftSubsystem.getLeftEncoderDistance() < 0.2) &&
           (liftSubsystem.getRightEncoderDistance() < 0.2)) {

            climbStep = 11;

        } else {

            angleSubsystem.enablePID(false);  // disable the angle pid 
            angleSubsystem.setPower(0);  // zero the power to the angle

            liftSubsystem.enablePID(true);  // enable lift pid
            liftSubsystem.setPosition(-0.5);  // set setpoint
        }
        break;

      case 11:  // latch

        hookSubsystem.set(true);  // extend hooks
        timer0.reset();  // reset timer
        timer0.start();  // start timer
        climbStep = 12;  // move to next step
        break;

      case 12:  // kill power to winch
        
        if (timer0.get() > 0.5) {  // wait half a second
          timer0.stop();  // stop the timer
          liftSubsystem.enablePID(false);  // disable the winch pid
          liftSubsystem.setPower(0);  // zero the power to winch motors
          climbStep = 13;  // move to next step
        }
        break;

      case 13:

        commandDone = true;  // we're done
        climbStep = 1;  // make loop do nothing
        break;

    }

    // safe mode trigger
    // if we're exceeding current limit:
    if(liftSubsystem.getLeftMotorCurrent() > Constants.Climb.lift.maxCurrentThreshold || 
      liftSubsystem.getRightMotorCurrent() > Constants.Climb.lift.maxCurrentThreshold) {

        if (currentLimitTimeout.get() > Constants.Climb.lift.maxCurrentTimeout) {  // if the timeout has expired while the current is exceeded
          currentLimitTimeout.stop(); // stop the timeout timer
          climbStep = 0; // go to safe mode!
        }

    } else {  // if the current has not exceeded threshold
      currentLimitTimeout.reset();  // full reset the timer
      currentLimitTimeout.start();
    }

  }

  @Override
  public void end(boolean interrupted) {

  }

  // TODO: FIX THIS
  @Override
  public boolean isFinished() {
    return commandDone;
    // return false;  // force never-ending for testing
  }
}
