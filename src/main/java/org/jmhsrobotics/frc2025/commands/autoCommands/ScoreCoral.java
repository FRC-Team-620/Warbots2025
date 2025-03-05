package org.jmhsrobotics.frc2025.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;

public class ScoreCoral extends Command {
  private Intake intake;

  public ScoreCoral(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.set(Constants.IntakeConstants.kCoralIntakeSpeedDutyCycle / 2.0);
  }

  @Override
  public void execute() {
    intake.set(Constants.IntakeConstants.kCoralIntakeSpeedDutyCycle / 2.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intake.set(0);
  }
}
