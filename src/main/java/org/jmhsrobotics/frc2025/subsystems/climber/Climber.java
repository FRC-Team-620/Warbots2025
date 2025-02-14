package org.jmhsrobotics.frc2025.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.climber.indexer.IndexerIO;
import org.jmhsrobotics.frc2025.subsystems.climber.indexer.IndexerIOInputsAutoLogged;

public class Climber extends SubsystemBase {
  private ClimberIO climberIO;
  private ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();
  private IndexerIO indexerIO;
  private IndexerIOInputsAutoLogged indexerInputs = new IndexerIOInputsAutoLogged();
  private double indexerSetpointDegrees;

  public Climber(ClimberIO climberIO, IndexerIO indexerIO) {
    this.climberIO = climberIO;
    this.indexerIO = indexerIO;
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(climberInputs);
    indexerIO.updateInputs(indexerInputs);
  }

  public void setClimberSpeed(double climberSpeedDutyCycle) {
    climberIO.set(climberSpeedDutyCycle);
  }

  public void setIndexerSetpoint(double indexerSetpointDegrees) {
    this.indexerSetpointDegrees = indexerSetpointDegrees;
    indexerIO.setPositionDegrees(indexerSetpointDegrees);
  }

  public double getClimberPositionDegrees() {
    return climberInputs.positionDegrees;
  }

  public double getIndexerPositionDegrees() {
    return indexerInputs.positionDegrees;
  }

  public boolean indexerAtGoal() {
    return Math.abs(this.indexerSetpointDegrees - indexerInputs.positionDegrees)
        < Constants.IndexerConstants.kAngleTolerance;
  }

  public boolean climberAtMax() {
    return climberInputs.positionDegrees >= Constants.ClimberConstants.kMaxDegrees;
  }
}
