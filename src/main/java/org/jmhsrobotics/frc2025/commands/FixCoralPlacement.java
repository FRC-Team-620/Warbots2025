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

  private Timer timer = new Timer();

  private boolean hasPassedSensor = false;

  private Debouncer debouncer = new Debouncer(0.03, DebounceType.kBoth);
  private boolean coralInIntake;

  public FixCoralPlacement(Intake intake, Wrist wrist) {
    this.intake = intake;
    this.wrist = wrist;

    addRequirements(intake, wrist);
  }

  @Override
  public void initialize() {
    intake.set(Constants.IntakeConstants.kCoralDefaultCommandSpeed);
    timer.reset();
  }

  @Override
  public void execute() {
    this.coralInIntake = debouncer.calculate(intake.isCoralInIntake());
    if (this.coralInIntake && !hasPassedSensor) {
      intake.set(Constants.IntakeConstants.kCoralDefaultCommandSpeed);
    } else {
      hasPassedSensor = true;
      intake.set(-Constants.IntakeConstants.kCoralDefaultCommandSpeed);
    }

    if (hasPassedSensor && coralInIntake) timer.start();
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
