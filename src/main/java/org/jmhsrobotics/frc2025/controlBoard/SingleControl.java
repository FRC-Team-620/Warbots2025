package org.jmhsrobotics.frc2025.controlBoard;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.util.ControllerMonitor;

public class SingleControl implements ControlBoard {
  CommandXboxController driver = new CommandXboxController(0);
  private Intake intake;
  private Elevator elevator;

  private Trigger nop =
      new Trigger(
          () -> {
            return false;
          });

  public SingleControl(Intake intake, Elevator elevator) {
    ControllerMonitor.addController(driver.getHID(), "Driver");
    this.intake = intake;
    this.elevator = elevator;
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

  // both aligns return 0 because buttoms are available
  @Override
  public double alignLeft() {
    return 0;
  }

  @Override
  public double alignRight() {
    return 0;
  }

  @Override
  public Trigger autoAlignBarge() {
    return nop;
  }

  public Trigger autoAlignAlgaeIntake() {
    return nop;
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

  @Override
  public Trigger L1AutoAlign() {
    return nop;
  }

  @Override
  public Trigger TeleopAutoScore() {
    return nop;
  }

  // =======Operator Controls=======

  @Override
  public Trigger intakeCoralFromIndexer() {
    return driver.rightStick().and(searchMode);
  }

  @Override
  public DoubleSupplier intakeCoral() {
    // return driver.leftTrigger();
    return () -> driver.getLeftTriggerAxis();
  }

  @Override
  public DoubleSupplier extakeCoral() {
    return () -> driver.getRightTriggerAxis();
  }

  @Override
  public Trigger placeCoralLevel1() {
    // if (intake.getMode() != Constants.ModeConstants.kCoral) return nop;
    return driver.a().and(coralMode);
  }

  @Override
  public Trigger placeCoralLevel2() {
    // if (intake.getMode() != Constants.ModeConstants.kCoral) return nop;
    return driver.b().and(coralMode);
  }

  @Override
  public Trigger placeCoralLevel3() {
    // if (intake.getMode() != Constants.ModeConstants.kCoral) return nop;
    return driver.x().and(coralMode);
  }

  @Override
  public Trigger placeCoralLevel4() {
    // if (intake.getMode() != Constants.ModeConstants.kCoral) return nop;
    return driver.y().and(coralMode);
  }

  @Override
  public Trigger scoreAlgaeProcesser() {
    return driver.a().and(algaeMode);
  }

  @Override
  public Trigger algaeIntermediateSetpoint() {
    return driver.b().and(algaeMode);
  }

  @Override
  public Trigger prepareAlgaeBarge() {
    return driver.x().and(algaeMode);
  }

  @Override
  public Trigger scoreAlgaeBarge() {
    return driver.y().and(algaeMode);
  }

  @Override
  public Trigger elevatorIntakeCoral() {
    // if (intake.getMode() != Constants.ModeConstants.kSearch) return nop;
    return driver.a().and(searchMode);
  }

  @Override
  public Trigger takeAlgaeLevel2() {
    // if (intake.getMode() != Constants.ModeConstants.kSearch) return nop;
    return driver.b().and(searchMode);
  }

  @Override
  public Trigger takeAlgaeLevel3() {
    // if (intake.getMode() != Constants.ModeConstants.kSearch) return nop;
    return driver.x().and(searchMode);
  }

  @Override
  public Trigger takeAlgaeQTip() {
    // if (intake.getMode() != Constants.ModeConstants.kSearch) return nop;
    return driver.y().and(searchMode);
  }

  @Override
  public Trigger changeModeLeft() {
    return driver.back();
  }

  @Override
  public Trigger changeModeRight() {
    return driver.start();
  }

  @Override
  public Trigger UnOverrideControlMode() {
    return driver.start().and(driver.back());
  }

  @Override
  public Trigger zeroElevator() {
    return driver.leftStick();
  }

  @Override
  public Trigger skipAutoScoreEast() {
    return nop;
  }

  @Override
  public Trigger skipAutoScoreWest() {
    return nop;
  }

  @Override
  public Trigger revertAutoScoreEast() {
    return nop;
  }

  @Override
  public Trigger revertAutoScoreWest() {
    return nop;
  }
}
