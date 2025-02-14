package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;

import org.jmhsrobotics.frc2025.subsystems.climber.Climber;

public class IndexerMove extends Command {
  private Climber climber;
  private double indexerAngleDegrees;

  public IndexerMove(Climber climber, double indexerAngleDegrees) {
    this.climber = climber;
    this.indexerAngleDegrees = indexerAngleDegrees;

    addRequirements(climber);
  }

  @Override
  public void execute() {
    this.climber.setIndexerSetpoint(indexerAngleDegrees);
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return this.climber.indexerAtGoal();
  }
}
