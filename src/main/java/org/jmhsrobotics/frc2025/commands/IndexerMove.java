package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.indexer.Indexer;

public class IndexerMove extends Command {
  private Indexer indexer;

  public IndexerMove(Indexer indexer) {
    this.indexer = indexer;
  }

  @Override
  public void initialize() {
    if (indexer.getSetpointDegrees() == Constants.IndexerConstants.kRotationDownDegrees)
      indexer.setSetpoint(Constants.IndexerConstants.kRotationUpDegrees);
    else indexer.setSetpoint(Constants.IndexerConstants.kRotationDownDegrees);
  }

  @Override
  public boolean isFinished() {
    return indexer.atGoal();
  }
}
