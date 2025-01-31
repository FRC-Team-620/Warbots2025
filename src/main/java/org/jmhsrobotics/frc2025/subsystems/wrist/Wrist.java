package org.jmhsrobotics.frc2025.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jmhsrobotics.frc2025.Constants;

public class Wrist extends SubsystemBase {
  private WristIO wristIO;
  private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  public Wrist(WristIO wristIO) {
    this.wristIO = wristIO;
  }

  @Override
  public void periodic() {
    wristIO.updateInputs(inputs);
  }

  public boolean atGoal(double setPoint) {
    return Math.abs(setPoint - inputs.positionDegrees) < Constants.WristConstants.kAngleTolerance;
  }

  public void setSetpoint(double setPoint) {
    wristIO.setPositionDegrees(setPoint);
  }
}
