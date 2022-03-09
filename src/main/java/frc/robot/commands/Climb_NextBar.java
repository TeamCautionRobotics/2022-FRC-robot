package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ClimbAngle;
import frc.robot.subsystems.ClimbHook;
import frc.robot.subsystems.ClimbLift;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

public class Climb_NextBar extends CommandBase {
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
  public Climb_NextBar(ClimbAngle angleSubsystem, ClimbHook hookSubsystem, ClimbLift liftSubsystem) {
    
    addRequirements(angleSubsystem, hookSubsystem, liftSubsystem);

    this.angleSubsystem = angleSubsystem;
    this.hookSubsystem = hookSubsystem;
    this.liftSubsystem = liftSubsystem;

  }

  @Override
  public void initialize() {

    commandDone = false;  // wpilib bug workaround

    hookSubsystem.set(false);  // retract the static hooks
    angleSubsystem.setNeutralMode(NeutralMode.Coast);  // coast the arm motors
    liftSubsystem.setIdleMode(IdleMode.kBrake);  // brake the lift motors

    climbStep = 10;  // start on step 10

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

      case 10:  // winch out

        // if we're at the setpoint
        if (liftSubsystem.getLeftLiftPosition() > 27.8 &&
            liftSubsystem.getRightLiftPosition() > 27.8) {

              climbStep = 11;

        } else {  // if we're not there yet

          // kill power to angle motors
          angleSubsystem.enablePID(false);
          angleSubsystem.setPower(0);

          // go 28 inches out
          liftSubsystem.setPosition(28);
          liftSubsystem.enablePID(true);

        }
        break;
      
      case 11:  // angle out

        // if we're at the setpoint
        if (angleSubsystem.getLeftEncoderDistance() > 110 &&
            angleSubsystem.getRightEncoderDistance() > 110) {

          // hookSubsystem.set(false);  // retract the hooks
          climbStep = 15;

        } else {  // if we're not there yet

          angleSubsystem.setPosition(120);
          angleSubsystem.enablePID(true);

        }
        break;

      case 12:  // go time! pull down

        // if we're at the setpoint
        if (liftSubsystem.getLeftLiftPosition() < -0.2 &&
            liftSubsystem.getRightLiftPosition() < -0.2) {

          climbStep = 13;

        } else {  // if we're not there yet

          // make arms go limp
          angleSubsystem.enablePID(false);
          angleSubsystem.setPower(0);

          // pull down
          liftSubsystem.setPosition(-1.0);
          liftSubsystem.enablePID(true);

        }
        break;

      case 13:  // extend the hooks and begin the timer
        
        hookSubsystem.set(true);
        timer0.stop();
        timer0.reset();
        timer0.start();
        climbStep = 14;
        break;

      case 14:  // release hold

        if (timer0.get() > 0.5) {  // half-second timer before power release

          // kill power to winch
          liftSubsystem.enablePID(false);
          liftSubsystem.setPower(0);
          timer0.stop();
          climbStep = 15;

        }
        break;

      case 15:  // finish
        
        commandDone = true;
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
    // return commandDone;
    return false;  // force never-ending for testing
  }
}
