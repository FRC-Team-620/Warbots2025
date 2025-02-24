package org.jmhsrobotics.frc2025.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private ClimberIO climberIO;
  private ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO climberIO) {
    this.climberIO = climberIO;
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(climberInputs);
  }

  public void set(double climberSpeedDutyCycle) {
    climberIO.set(climberSpeedDutyCycle);
  }

  public double getClimberPositionDegrees() {
    return climberInputs.positionDegrees;
  }

  public void setBrakeMode(boolean enable) {
    climberIO.setBrakeMode(enable);
  }
}
