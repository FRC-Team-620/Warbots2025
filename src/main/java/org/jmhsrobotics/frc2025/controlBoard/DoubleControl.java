package org.jmhsrobotics.frc2025.controlBoard;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.util.ControllerMonitor;
import org.littletonrobotics.junction.Logger;

public class DoubleControl implements ControlBoard {
  public CommandXboxController driver = new CommandXboxController(0);
  public CommandXboxController operator = new CommandXboxController(1);
  private Intake intake;
  private Elevator elevator;

  public DoubleControl(Intake intake, Elevator elevator) {
    ControllerMonitor.addController(this.operator.getHID(), "Operator");
    ControllerMonitor.addController(this.driver.getHID(), "Driver");
    this.intake = intake;
    this.elevator = elevator;
    coralMode.whileTrue(
        new InstantCommand(
                () -> {
                  Logger.recordOutput("ctlMode", "coral");
                })
            .ignoringDisable(true));
    algaeMode.whileTrue(
        new InstantCommand(
                () -> {
                  Logger.recordOutput("ctlMode", "algae");
                })
            .ignoringDisable(true));
    searchMode.whileTrue(
        new InstantCommand(
                () -> {
                  Logger.recordOutput("ctlMode", "search");
                })
            .ignoringDisable(true));
  }

  private Trigger coralMode =
      new Trigger(
          () -> {
            return intake.getMode() == Constants.ModeConstants.kCoral;
          });

  private Trigger algaeMode =
      new Trigger(
          () -> {
            return intake.getMode() == Constants.ModeConstants.kAlgae;
          });

  private Trigger searchMode =
      new Trigger(
          () -> {
            return intake.getMode() == Constants.ModeConstants.kSearch;
          });

  private Trigger elevatorAtBottom =
      new Trigger(
          () -> {
            return elevator.getSetpoint() == Constants.ElevatorConstants.kCoralIntakeMeters;
          });

  // ========Driver Controls========

  @Override
  public double rotation() {
    return driver.getRightX();
  }

  @Override
  public double translationX() {
    return driver.getLeftX();
  }

  @Override
  public double translationY() {
    return driver.getLeftY();
  }

  @Override
  public double alignLeft() {
    return driver.getLeftTriggerAxis();
  }

  @Override
  public double alignRight() {
    return driver.getRightTriggerAxis();
  }

  @Override
  public Trigger autoAlignBarge() {
    return driver.b().and(algaeMode);
  }

  public Trigger autoAlignAlgaeIntake() {
    return driver.a().and(searchMode);
  }

  @Override
  public Trigger resetForward() {
    return driver.rightBumper();
  }

  @Override
  public Trigger turboMode() {
    return driver.leftBumper();
  }

  @Override
  public Trigger AdjustAlignBargeLeft() {
    return driver.povLeft();
  }

  @Override
  public Trigger AdjustAlignBargeRight() {
    return driver.povRight();
  }

  // =======Operator Controls=======

  @Override
  public Trigger intakeCoralFromIndexer() {
    return operator.rightBumper();
  }

  @Override
  public DoubleSupplier intakeCoral() {
    return () -> operator.getLeftTriggerAxis();
  }

  @Override
  public DoubleSupplier extakeCoral() {
    return () -> operator.getRightTriggerAxis();
  }

  @Override
  public Trigger placeCoralLevel1() {
    return operator.a().and(coralMode);
  }

  @Override
  public Trigger placeCoralLevel2() {
    return operator.b().and(coralMode);
  }

  @Override
  public Trigger placeCoralLevel3() {
    return operator.x().and(coralMode);
  }

  @Override
  public Trigger placeCoralLevel4() {
    return operator.y().and(coralMode);
  }

  @Override
  public Trigger scoreAlgaeProcesser() {
    return (operator.a().or(operator.b())).and(algaeMode);
  }

  @Override
  public Trigger scoreAlgaeBarge() {
    return (operator.y().or(operator.x())).and(algaeMode);
  }

  @Override
  public Trigger elevatorIntakeCoral() {
    return operator.a().and(searchMode);
  }

  @Override
  public Trigger takeAlgaeLevel2() {
    return operator.b().and(searchMode);
  }

  @Override
  public Trigger takeAlgaeLevel3() {
    return operator.x().and(searchMode);
  }

  @Override
  public Trigger takeAlgaeQTip() {
    return operator.y().and(searchMode);
  }

  @Override
  public Trigger changeModeLeft() {
    return operator.leftStick();
  }

  @Override
  public Trigger changeModeRight() {
    return operator.rightStick();
  }

  @Override
  public Trigger UnOverrideControlMode() {
    return operator.rightStick().and(operator.leftStick());
  }

  @Override
  public Trigger zeroElevator() {
    return operator.back();
  }
}
