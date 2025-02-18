package org.jmhsrobotics.frc2025.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jmhsrobotics.frc2025.Constants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorIO elevatorIO;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private MechanismLigament2d stage1 = // TODO move to Robot Container
      new MechanismLigament2d("stage1", 3, 90, 10, new Color8Bit(0, 0, 168));
  private MechanismLigament2d carriage =
      new MechanismLigament2d("carriage", 3, 0, 5, new Color8Bit(255, 0, 0));
  Mechanism2d elevatorMech = new Mechanism2d(4, 4);
  private double setPointMeters;

  private TrapezoidProfile trapezoidProfile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              Constants.ElevatorConstants.kTrapezoidalProfileMaxVelocity,
              Constants.ElevatorConstants.kTrapezoidalProfileMaxAcceleration));

  private boolean closedLoopMode = true;

  public Elevator(ElevatorIO elevatorIO) {
    this.elevatorIO = elevatorIO;
    var root = elevatorMech.getRoot("base", 1, 0);
    root.append(stage1).append(carriage);
    SmartDashboard.putData("Elevator", elevatorMech);
  }

  @Override
  public void periodic() {
    elevatorIO.updateInputs(inputs);
    stage1.setLength(inputs.heightMeters / 2);
    carriage.setLength(inputs.heightMeters / 2);

    TrapezoidProfile.State calculatedState =
        trapezoidProfile.calculate(
            0.02,
            new TrapezoidProfile.State(inputs.heightMeters, inputs.velocityMPS),
            new TrapezoidProfile.State(this.setPointMeters, 0));

    Logger.recordOutput("Elevator/Current", this.getCurrentAmps());
    Logger.recordOutput("Elevator/Height", inputs.heightMeters);
    Logger.recordOutput("Elevator/Goal Meters", setPointMeters);
    Logger.recordOutput("Elevator/SpeedMPS", inputs.velocityMPS);
    Logger.recordOutput("Elevator/AccelerationMPSS", inputs.accelerationMPSS);
    Logger.recordOutput("Elevator/Trapezoidal Profile Setpoint", calculatedState.position);

    if (closedLoopMode) {
      if (calculatedState.position < 1.8 && calculatedState.position > 0) {
        elevatorIO.setPositionMeters(calculatedState.position);
      }
    }
  }

  public boolean atGoal() {
    return Math.abs(inputs.heightMeters - this.setPointMeters)
        < Constants.ElevatorConstants.kHeightTolerance;
  }

  public void setSetpoint(double setPoint) {
    this.closedLoopMode = true;
    this.setPointMeters = setPoint;
    // elevatorIO.setPositionMeters(setPoint);
  }

  public void setVoltage(double voltage) {
    this.closedLoopMode = false;
    elevatorIO.setVoltage(voltage);
  }

  public double getHeight() {
    return inputs.heightMeters;
  }

  public double getVelocity() {
    return inputs.velocityMPS;
  }

  public double getCurrentAmps() {
    double totalAmps = 0;
    for (int i = 0; i < inputs.motorAmps.length; i++) {
      totalAmps += inputs.motorAmps[i];
    }
    return totalAmps;
  }

  public void setZero() {
    elevatorIO.setZero();
  }

  public void setBrakeMode(boolean enable) {
    elevatorIO.setBrakeMode(enable);
  }

  public double getSetpoint() {
    return setPointMeters;
  }
}
