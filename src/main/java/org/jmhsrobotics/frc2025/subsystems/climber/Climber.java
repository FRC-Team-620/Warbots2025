package org.jmhsrobotics.frc2025.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private ClimberIO climberIO;
  private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO climberIO) {
    this.climberIO = climberIO;
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(inputs);
    Logger.recordOutput("Climber/Output Current", inputs.motorAmps);
    Logger.recordOutput("Climber/Motor Speed", inputs.motorRPM);
  }

  public void setSpeedDutyCycle(double climberSpeedDutyCycle) {
    climberIO.set(climberSpeedDutyCycle);
  }

  public double getClimberPositionDegrees() {
    return inputs.positionDegrees;
  }

  public void setBrakeMode(boolean enable) {
    climberIO.setBrakeMode(enable);
  }
}
