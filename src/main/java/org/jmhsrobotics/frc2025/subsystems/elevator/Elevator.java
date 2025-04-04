package org.jmhsrobotics.frc2025.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.util.CheckTolerance;
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

  private boolean isOpenLoop = false;

  private State calculatedState = new State(Constants.ElevatorConstants.kLevel1Meters, 0);
  private TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new Constraints(1000, 14));

  public Elevator(ElevatorIO elevatorIO) {
    this.elevatorIO = elevatorIO;
    var root = elevatorMech.getRoot("base", 1, 0);
    root.append(stage1).append(carriage);
    SmartDashboard.putData("Elevator", elevatorMech);
  }

  @Override
  public void periodic() {
    calculatedState =
        trapezoidProfile.calculate(0.02, calculatedState, new State(this.setPointMeters, 0));
    if (!this.isOpenLoop) elevatorIO.setPositionMeters(calculatedState.position);

    elevatorIO.updateInputs(inputs);
    stage1.setLength(inputs.heightMeters / 2);
    carriage.setLength(inputs.heightMeters / 2);

    Logger.recordOutput("Elevator/Current", this.getCurrentAmps());
    Logger.recordOutput("Elevator/Height", inputs.heightMeters);
    Logger.recordOutput("Elevator/Setpoint Value", setPointMeters);
    Logger.recordOutput("Elevator/Calculated Setpoint", calculatedState.position);
    Logger.recordOutput("Elevator/VelocityDegPerSec", inputs.elevatorSpeedCmPerSec);
    Logger.recordOutput("Elevator/Is Open Loop", this.isOpenLoop);
    SmartDashboard.putNumber("Elevator/Raw Height Meters", inputs.heightMeters);
  }

  public void setSetpoint(double setPoint) {
    this.isOpenLoop = false;
    this.setPointMeters = setPoint;
  }

  public void setVoltage(double voltage) {
    this.isOpenLoop = true;
    elevatorIO.setVoltage(voltage);
  }

  public double getHeight() {
    return inputs.heightMeters;
  }

  public boolean atGoal() {
    return CheckTolerance.atGoalTolerance(
        setPointMeters, inputs.heightMeters, Constants.ElevatorConstants.kHeightTolerance);
  }

  public double getVelocity() {
    return inputs.velocityMPS;
  }

  public double getCurrentAmps() {
    double totalAmps = 0;
    if (inputs.motorAmps == null) {
      return 0;
    }
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
