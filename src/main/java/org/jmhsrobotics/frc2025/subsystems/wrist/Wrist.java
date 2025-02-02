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

  public boolean atGoal() {
    return Math.abs(wristIO.getSetpoint() - inputs.positionDegrees)
        < Constants.WristConstants.kAngleTolerance;
  }

  public double getPositionDegrees() {
    return inputs.positionDegrees;
  }

  public void setSetpoint(double setPoint) {
    wristIO.setPositionDegrees(setPoint);
  }
}
