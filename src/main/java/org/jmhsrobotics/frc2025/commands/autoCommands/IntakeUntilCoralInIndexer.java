package org.jmhsrobotics.frc2025.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.indexer.Indexer;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class IntakeUntilCoralInIndexer extends Command {
  private Wrist wrist;
  private Intake intake;
  private Indexer indexer;

  public IntakeUntilCoralInIndexer(Wrist wrist, Intake intake, Indexer indexer) {
    this.wrist = wrist;
    this.intake = intake;
    this.indexer = indexer;

    addRequirements(wrist, intake, indexer);
  }

  @Override
  public void initialize() {
    indexer.set(Constants.IndexerConstants.kIndexerSpeedDutyCycle);
    intake.set(Constants.IntakeConstants.kCoralIntakeIndexerSpeedDutyCycle);
    wrist.setSetpoint(Constants.WristConstants.kRotationIntakeCoralDegrees);
  }

  @Override
  public void execute() {
    indexer.set(Constants.IndexerConstants.kIndexerSpeedDutyCycle);
    intake.set(Constants.IntakeConstants.kCoralIntakeIndexerSpeedDutyCycle);
  }

  @Override
  public boolean isFinished() {
    return indexer.hasCoral() || intake.isCoralInIntake();
  }

  @Override
  public void end(boolean interrupted) {
    indexer.set(Constants.IndexerConstants.kIndexerSpeedDutyCycle);
    intake.set(Constants.IntakeConstants.kCoralIntakeIndexerSpeedDutyCycle);
  }
}
