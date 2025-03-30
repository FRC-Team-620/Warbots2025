package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.indexer.Indexer;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;

public class IndexerMove extends Command {
  private Indexer indexer;
  private Intake intake;

  public IndexerMove(Indexer indexer, Intake intake) {
    this.indexer = indexer;
    this.intake = intake;

    addRequirements(indexer);
  }

  @Override
  public void initialize() {
    indexer.set(0);
  }

  @Override
  public void execute() {
    indexer.set(
        Constants.IndexerConstants
            .kIndexerSpeedDutyCycle); // make a constant for indexer speed duty cylce
  }

  @Override
  public boolean isFinished() {
    // return intake.isCoralInIntake();
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    indexer.set(0);
    System.out.println("Indexer Move Complete: " + interrupted);
  }
}
