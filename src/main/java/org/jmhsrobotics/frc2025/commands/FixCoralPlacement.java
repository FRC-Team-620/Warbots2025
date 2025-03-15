package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class FixCoralPlacement extends Command {
  private Intake intake;
  private Wrist wrist;

  private boolean hasPassedSensor = false;
  private Timer timer = new Timer();
  private Timer coralRetractTimer = new Timer();

  private Debouncer debouncer = new Debouncer(0.15, DebounceType.kRising);
  private boolean coralInIntake;

  private boolean hasResetPlacementCommandTimer = false;

  public FixCoralPlacement(Intake intake, Wrist wrist) {
    this.intake = intake;
    this.wrist = wrist;

    addRequirements(intake, wrist);
  }

  @Override
  public void initialize() {
    this.timer.reset();
    this.coralRetractTimer.reset();
    this.hasPassedSensor = false;
    this.coralInIntake = true;

    hasResetPlacementCommandTimer = false;

    // tells intake that command is running which keeps it in coral mode without coral in front of
    // sensor
    intake.set(Constants.IntakeConstants.kCoralDefaultCommandSpeed * 0.9);
  }

  @Override
  public void execute() {
    this.coralInIntake = debouncer.calculate(intake.isCoralInIntake());
    if (intake.isCoralInIntake() && !hasPassedSensor) {
      intake.set(Constants.IntakeConstants.kCoralDefaultCommandSpeed);
    } else {
      hasPassedSensor = true;
      this.timer.start();
      wrist.setSetpoint(Constants.WristConstants.kSafeAngleDegrees);
      intake.set(-Constants.IntakeConstants.kCoralDefaultCommandSpeed);
    }

    if (hasPassedSensor == true && this.hasResetPlacementCommandTimer == false) {
      intake.startPlacementCommandTimer();
      this.hasResetPlacementCommandTimer = true;
    }
  }

  @Override
  public boolean isFinished() {
    // once coral comes back in from of sensor after passing
    // TODO: make the threshold for the time cancel accurate
    return (hasPassedSensor && coralInIntake) || timer.hasElapsed(3);
  }

  @Override
  public void end(boolean interrupted) {
    coralRetractTimer.reset();
    intake.set(0);
    System.out.println("Starting Intake from indexer");
  }
}
