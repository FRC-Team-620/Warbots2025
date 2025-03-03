package org.jmhsrobotics.frc2025.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jmhsrobotics.frc2025.Constants;
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
    Logger.recordOutput("Climber/Position Degrees", inputs.positionDegrees);
  }

  public void setSpeedDutyCycle(double climberSpeedDutyCycle) {
    if (inputs.positionDegrees < Constants.ClimberConstants.kSoftLimitTopDegrees)
      climberIO.set(MathUtil.clamp(climberSpeedDutyCycle, -1, 0));
    else if (inputs.positionDegrees > Constants.ClimberConstants.kSoftLimitBottomDegrees)
      climberIO.set(MathUtil.clamp(climberSpeedDutyCycle, 0, 1));
    else climberIO.set(climberSpeedDutyCycle);
  }

  public double getClimberPositionDegrees() {
    return inputs.positionDegrees;
  }

  public void setBrakeMode(boolean enable) {
    climberIO.setBrakeMode(enable);
  }
}
