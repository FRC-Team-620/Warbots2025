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

  private Debouncer debouncer = new Debouncer(0.05, DebounceType.kBoth);
  private boolean coralInIntake;

  public FixCoralPlacement(Intake intake, Wrist wrist) {
    this.intake = intake;
    this.wrist = wrist;
    System.out.println("========Starting Intake Placement Command");

    addRequirements(intake, wrist);
  }

  @Override
  public void initialize() {
    intake.set(Constants.IntakeConstants.kCoralDefaultCommandSpeed);
  }

  @Override
  public void execute() {
    System.out.println("+++++Running placement Command");
    this.coralInIntake = debouncer.calculate(intake.isCoralInIntake());
    if (this.coralInIntake && !hasPassedSensor) {
      intake.set(Constants.IntakeConstants.kCoralDefaultCommandSpeed);
    } else {
      hasPassedSensor = true;
      intake.set(-Constants.IntakeConstants.kCoralDefaultCommandSpeed);
    }
  }

  @Override
  public boolean isFinished() {
    // once coral comes back in from of sensor after passing
    return hasPassedSensor && coralInIntake;
  }

  @Override
  public void end(boolean interrupted) {
    intake.set(0);
    wrist.setSetpoint(Constants.WristConstants.kSafeAngleDegrees);
  }
}
