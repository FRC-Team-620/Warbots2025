package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.indexer.Indexer;

public class IndexerMove extends Command {
  private Indexer indexer;
  private double setPointDegrees;

  public IndexerMove(Indexer indexer, double setPointDegrees) {
    this.indexer = indexer;
    this.setPointDegrees = setPointDegrees;

    addRequirements(indexer);
  }

  @Override
  public void initialize() {
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
