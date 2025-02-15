package org.jmhsrobotics.frc2025.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jmhsrobotics.frc2025.subsystems.climber.indexer.IndexerIO;
import org.jmhsrobotics.frc2025.subsystems.climber.indexer.IndexerIOInputsAutoLogged;

public class Climber extends SubsystemBase {
  private ClimberIO climberIO;
  private ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();
  private IndexerIO indexerIO;
  private IndexerIOInputsAutoLogged indexerInputs = new IndexerIOInputsAutoLogged();

  public Climber(ClimberIO climberIO, IndexerIO indexerIO) {
    this.climberIO = climberIO;
    this.indexerIO = indexerIO;
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(climberInputs);
    indexerIO.updateInputs(indexerInputs);
  }

  public void set(double climberSpeedDutyCycle, double indexerSetPointDegrees) {
    climberIO.set(climberSpeedDutyCycle);
    indexerIO.setPositionDegrees(indexerSetPointDegrees);
  }

  public double getClimberPositionDegrees() {
    return climberInputs.positionDegrees;
  }

  public double getIndexerPositionDegrees() {
    return indexerInputs.positionDegrees;
  }

  public void setBrakeMode(boolean enable) {
    climberIO.setBrakeMode(enable);
  }
}
