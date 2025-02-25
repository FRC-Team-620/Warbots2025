package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class IntakeFromIndexer extends Command {
  private Wrist wrist;
  private Intake intake;

  public IntakeFromIndexer(Wrist wrist, Intake intake) {
    this.wrist = wrist;
    this.intake = intake;

    addRequirements(wrist, intake);
  }

  @Override
  public void initialize() {
    intake.set(0);
    wrist.setSetpoint(Constants.WristConstants.kRotationIntakeCoralDegrees);
  }

  @Override
  public void execute() {
    if (intake.getCoralDistance() > 20)
      intake.set(Constants.IntakeConstants.kCoralIntakeIndexerSpeedDutyCycle);
    else intake.set(Constants.IntakeConstants.kCoralIntakeIndexerSlowSpeedDutyCycle);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intake.set(0);
    wrist.setSetpoint(Constants.WristConstants.kSafeAngleDegrees);
  }
}
