package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.indexer.Indexer;

public class IndexerMove extends Command {
  private Indexer indexer;
  private double setPointDegrees;

  public IndexerMove(Indexer indexer) {
    this.indexer = indexer;

    addRequirements(indexer);
  }

  @Override
  public void initialize() {
    if (this.indexer.getSetPointDegrees() == Constants.IndexerConstants.kRotationDownDegrees) {
      this.setPointDegrees = Constants.IndexerConstants.kRotationUpDegrees;
    } else {
      this.setPointDegrees = Constants.IndexerConstants.kRotationDownDegrees;
    }
    indexer.setSetpoint(setPointDegrees);
  }

  @Override
  public void execute() {
    indexer.setSetpoint(setPointDegrees);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(setPointDegrees - indexer.getPositionDegrees())
        < Constants.IndexerConstants.kTolerance;
  }
}
