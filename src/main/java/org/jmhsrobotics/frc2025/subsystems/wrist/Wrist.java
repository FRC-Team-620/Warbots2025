package org.jmhsrobotics.frc2025.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jmhsrobotics.frc2025.Constants;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private WristIO wristIO;
  private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private double setPointDegrees = Constants.WristConstants.kSafeAngleDegrees;

  public Wrist(WristIO wristIO) {
    this.wristIO = wristIO;
  }

  @Override
  public void periodic() {
    wristIO.updateInputs(inputs);
    Logger.recordOutput("Wrist/AngleDegrees", inputs.positionDegrees);
    Logger.recordOutput("Wrist/OutputCurrent", inputs.motorAmps);
    Logger.recordOutput("Wrist/GoalAngle", setPointDegrees);
  }

  public double getPositionDegrees() {
    return inputs.positionDegrees;
  }
  public boolean atGoal() {
    return GoalTolerance.atGoalTolerance(setPointDegrees, inputs.positionDegrees, Constants.WristConstants.kAngleTolerance);
  }

  public boolean checkWristSafe() {
    return this.getPositionDegrees() > 20;
  }

  public void setSetpoint(double setPoint) {
    this.setPointDegrees = setPoint;
    wristIO.setPositionDegrees(setPoint);
  }

  public void setBrakeMode(boolean enable) {
    wristIO.setBrakeMode(enable);
  }
}
