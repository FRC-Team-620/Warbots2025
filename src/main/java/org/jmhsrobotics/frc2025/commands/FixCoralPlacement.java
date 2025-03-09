package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class FixCoralPlacement extends Command {
  private Intake intake;
  private Wrist wrist;

  private boolean hasPassedSensor = false;

  private Debouncer debouncer = new Debouncer(0.15, DebounceType.kRising);
  private boolean coralInIntake;

  public FixCoralPlacement(Intake intake, Wrist wrist) {
    this.intake = intake;
    this.wrist = wrist;

    addRequirements(intake, wrist);
  }

  @Override
  public void initialize() {
    System.out.println("=====Starting to fix coral placement");
    this.hasPassedSensor = false;
    this.coralInIntake = true;
    intake.set(Constants.IntakeConstants.kCoralDefaultCommandSpeed * 0.9);
  }

  @Override
  public void execute() {
    this.coralInIntake = debouncer.calculate(intake.isCoralInIntake());
    if (this.coralInIntake && !hasPassedSensor) {
      intake.set(Constants.IntakeConstants.kCoralDefaultCommandSpeed * 0.75);
    } else {
      hasPassedSensor = true;
      wrist.setSetpoint(Constants.WristConstants.kSafeAngleDegrees);
      intake.set(-Constants.IntakeConstants.kCoralDefaultCommandSpeed);
    }
  }

  @Override
  public boolean isFinished() {
    // once coral comes back in from of sensor after passing
    return hasPassedSensor && coralInIntake;
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelIncoming;
  }

  @Override
  public void end(boolean interrupted) {
    intake.set(0);
    System.out.println("Starting Intake from indexer");
  }
}
