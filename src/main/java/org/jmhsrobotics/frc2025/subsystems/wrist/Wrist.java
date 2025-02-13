package org.jmhsrobotics.frc2025.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jmhsrobotics.frc2025.Constants;
import wristio

public class Wrist extends SubsystemBase {
  private WristIO wristIO;
  private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private double setPointDegrees;

  public Wrist(WristIO wristIO) {
    this.wristIO = wristIO;
  }

  @Override
  public void periodic() {
    wristIO.updateInputs(inputs);
  }

  public double getPositionDegrees() {
    return inputs.positionDegrees;
  }

  public boolean checkWristSafe() {
    return this.getPositionDegrees() > 20;
  }
  public double getSetpoint(){
    return setPointDegrees;
  }

  public void setSetpoint(double setPoint) {
    this.setPointDegrees = setPoint;
    wristIO.setPositionDegrees(setPoint);
  }
}
