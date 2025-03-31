package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;

public class FixCoralPlacement extends Command {
  private Intake intake;

  private Timer timer = new Timer();

  private Debouncer debouncer = new Debouncer(0.15, DebounceType.kRising);
  private boolean coralInIntake;

  public FixCoralPlacement(Intake intake) {
    this.intake = intake;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    this.timer.reset();
    this.coralInIntake = true;

    // tells intake that command is running which keeps it in coral mode without coral in front of
    // sensor
    intake.startPlacementCommandTimer();
    intake.set(-Constants.IntakeConstants.kCoralDefaultCommandSpeed * 1.2);
  }

  @Override
  public void execute() {
    this.coralInIntake = debouncer.calculate(intake.isCoralInIntake());

    intake.set(-Constants.IntakeConstants.kCoralDefaultCommandSpeed * 1.2);
  }

  @Override
  public boolean isFinished() {
    // once coral comes back in from of sensor after passing
    // TODO: make the threshold for the time cancel accurate
    return (this.coralInIntake) || timer.hasElapsed(3);
  }

  @Override
  public void end(boolean interrupted) {
    intake.set(0);
    System.out.println("Starting Intake from indexer");
  }
}
