package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class FixCoralPlacementOut extends Command {
  private Wrist wrist;
  private Intake intake;

  public FixCoralPlacementOut(Wrist wrist, Intake intake) {
    this.wrist = wrist;
    this.intake = intake;

    addRequirements(wrist, intake);
  }

  @Override
  public void initialize() {
    intake.set(Constants.IntakeConstants.kCoralDefaultCommandSpeed * 0.8);
    wrist.setSetpoint(Constants.WristConstants.kRotationIntakeCoralDegrees);
  }

  @Override
  public void execute() {
    intake.set(Constants.IntakeConstants.kCoralDefaultCommandSpeed * 0.8);
  }

  @Override
  public boolean isFinished() {
      return !intake.isCoralInIntake();
  }

  @Override
  public void end(boolean interrupted) {
      wrist.setSetpoint(Constants.WristConstants.kSafeAngleDegrees);
      intake.set(0);
  }
}
