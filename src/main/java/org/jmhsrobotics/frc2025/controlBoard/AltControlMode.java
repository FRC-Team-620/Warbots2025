package org.jmhsrobotics.frc2025.controlBoard;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;

public class AltControlMode implements ControlBoard {
  CommandXboxController driver = new CommandXboxController(0);
  private Intake intake;
  private Elevator elevator;

  private Trigger nop =
      new Trigger(
          () -> {
            return false;
          });

  public AltControlMode(Intake intake, Elevator elevator) {
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

  @Override
  public double rotation() {
    return driver.getRightX();
  }

  @Override
  public double translationX() {
    return calculateTotalTranslation(true);
  }

  @Override
  public double translationY() {
    return calculateTotalTranslation(false);
  }

  // normalizes the X and Y translations and puts them in a vector
  private double calculateTotalTranslation(boolean isX) {
    double driveX = driver.getLeftX();
    double driveY = driver.getLeftY();
    if (driveX == 0 && driveY == 0) {
      return 0;
    }
    Vector<N2> translationVector = VecBuilder.fill(driveX, driveY);
    translationVector = translationVector.unit();

    // multiply the normalized translation by the amount that the right trigger is
    // pressed
    // if(isX){
    // return translationVector.get(0) * driver.getRightTriggerAxis();
    // }
    return translationVector.get(isX ? 0 : 1) * driver.getRightTriggerAxis();
  }

  @Override
  public Trigger resetForward() {
    // return new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    return driver.rightBumper();
  }

  @Override
  public Trigger turboMode() {
    return nop;
  }

  // =======Operator Controls=======

  @Override
  public Trigger intakeCoralFromIndexer() {
    return driver.rightTrigger().and(elevatorAtBottom);
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
    // if (intake.getMode() != Constants.ModeConstants.kAlgae) return nop;
    return (driver.a().or(driver.b())).and(algaeMode);
  }

  @Override
  public Trigger scoreAlgaeBarge() {
    // if (intake.getMode() != Constants.ModeConstants.kAlgae) return nop;
    return (driver.y().or(driver.x())).and(algaeMode);
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
  public Trigger climbUp() {
    return driver.povUp();
  }

  @Override
  public Trigger climbDown() {
    return driver.povDown();
  }

  public Trigger moveIndexer() {
    return driver.povLeft();
  }

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
}
