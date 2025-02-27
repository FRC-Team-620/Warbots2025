package org.jmhsrobotics.frc2025.subsystems.wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jmhsrobotics.frc2025.Constants;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private WristIO wristIO;
  private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private double setPointDegrees = Constants.WristConstants.kSafeAngleDegrees;

  private TrapezoidProfile trapezoidProfile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              Constants.WristConstants.kMaxSpeedDPS,
              Constants.WristConstants.kMaxAccellerationDPSPS));

  public Wrist(WristIO wristIO) {
    this.wristIO = wristIO;
  }

  @Override
  public void periodic() {
    TrapezoidProfile.State calculatedState =
        trapezoidProfile.calculate(
            0.02,
            new TrapezoidProfile.State(inputs.positionDegrees, inputs.wristDPS),
            new TrapezoidProfile.State(this.setPointDegrees, 0));

    wristIO.updateInputs(inputs);
    Logger.recordOutput("Wrist/Angle Degrees", inputs.positionDegrees);
    Logger.recordOutput("Wrist/Output Current", inputs.motorAmps);
    Logger.recordOutput("Wrist/Goal Angle", setPointDegrees);
    Logger.recordOutput("Wrist/Calculated Setpoint", calculatedState.position);

    wristIO.setPositionDegrees(calculatedState.position);
  }

  public boolean atGoal() {
    return Math.abs(this.setPointDegrees - inputs.positionDegrees)
        < Constants.WristConstants.kAngleTolerance;
  }

  public double getPositionDegrees() {
    return inputs.positionDegrees;
  }

  public boolean checkWristSafe() {
    return this.getPositionDegrees() > 20;
  }

  public void setSetpoint(double setPoint) {
    this.setPointDegrees = setPoint;
  }

  public void setBrakeMode(boolean enable) {
    wristIO.setBrakeMode(enable);
  }

  public double getSetpoint() {
    return this.setPointDegrees;
  }
}
