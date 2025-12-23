package org.jmhsrobotics.frc2025.subsystems.wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.util.CheckTolerance;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private WristIO wristIO;
  private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private double setPointDegrees = Constants.WristConstants.kSafeAngleDegrees;

  private State calculatedState = new State(Constants.WristConstants.kSafeAngleDegrees, 0);
  private TrapezoidProfile trapezoidProfile =
      new TrapezoidProfile(new Constraints(36000, 360 * 80));

  public Wrist(WristIO wristIO) {
    this.wristIO = wristIO;
  }

  @Override
  public void periodic() {
    calculatedState =
        trapezoidProfile.calculate(0.02, calculatedState, new State(this.setPointDegrees, 0));
    wristIO.setPositionDegrees(calculatedState.position);

    wristIO.updateInputs(inputs);
    Logger.recordOutput("Wrist/AngleDegrees", inputs.positionDegrees);
    Logger.recordOutput("Wrist/OutputCurrent", inputs.motorAmps);
    Logger.recordOutput("Wrist/GoalAngle", setPointDegrees);
    Logger.recordOutput("Wrist/Calculated Setpoint", calculatedState.position);
    Logger.recordOutput("Wrist/MotorTemp", inputs.motorTemp);

    Logger.recordOutput("Wrist/VelocityDegPerSec", inputs.wristSpeedDegPerSec);
    Logger.recordOutput("Wrist/AccelerationRPM/S", inputs.wristAccelerationRPMPerSec);
  }

  public double getPositionDegrees() {
    return inputs.positionDegrees;
  }

  public boolean atGoal() {
    return CheckTolerance.atGoalTolerance(
        setPointDegrees, inputs.positionDegrees, Constants.WristConstants.kAngleTolerance);
  }

  public boolean checkWristSafe() {
    return this.getPositionDegrees() > 20;
  }

  public void setSetpoint(double setPoint) {
    this.setPointDegrees = setPoint;
    // wristIO.setPositionDegrees(setPoint);
  }

  public void setBrakeMode(boolean enable) {
    wristIO.setBrakeMode(enable);
  }

  public double getSetpoint() {
    return this.setPointDegrees;
  }
}
