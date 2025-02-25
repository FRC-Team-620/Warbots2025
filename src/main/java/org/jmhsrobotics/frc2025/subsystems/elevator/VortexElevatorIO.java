package org.jmhsrobotics.frc2025.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.util.SparkUtil;

public class VortexElevatorIO implements ElevatorIO {
  private SparkFlex vortexLeft =
      new SparkFlex(Constants.CAN.kElevatorMotorLeftID, MotorType.kBrushless);
  private SparkFlex vortexRight =
      new SparkFlex(Constants.CAN.kElevatorMotorRightID, MotorType.kBrushless);
  // private AbsoluteEncoder leftEncoder = vortexLeft.getAbsoluteEncoder();
  // private AbsoluteEncoder rightEncoder = vortexRight.getAbsoluteEncoder();
  private RelativeEncoder leftEncoder = vortexLeft.getEncoder();
  private RelativeEncoder rightEncoder = vortexRight.getEncoder();

  private SparkFlexConfig vortexLeftConfig;
  private SparkFlexConfig vortexRightConfig;
  private SparkClosedLoopController pidController;

  private boolean isOpenLoop = true;

  private double controlVoltage;

  private double goalMeters = 0;

  public VortexElevatorIO() {
    vortexLeftConfig = new SparkFlexConfig();
    vortexLeftConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .voltageCompensation(12)
        .inverted(true)
        .encoder
        .positionConversionFactor(Constants.ElevatorConstants.conversionFactor);
    vortexLeftConfig.closedLoop.pid(
        Constants.ElevatorConstants.kP,
        Constants.ElevatorConstants.kI,
        Constants.ElevatorConstants.kD);
    vortexLeftConfig
        .softLimit // TODO double check elevator limit directions
        .forwardSoftLimit(Constants.ElevatorConstants.kElevatorTopSoftLimit)
        .reverseSoftLimit(Constants.ElevatorConstants.kElevatorBottomSoftLimit)
        .forwardSoftLimitEnabled(Constants.ElevatorConstants.kElevatorLimitsEnabled)
        .reverseSoftLimitEnabled(Constants.ElevatorConstants.kElevatorLimitsEnabled);

    vortexRightConfig = new SparkFlexConfig();
    vortexRightConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .voltageCompensation(12)
        .follow(vortexLeft, true)
        .encoder
        .positionConversionFactor(Constants.ElevatorConstants.conversionFactor);
    vortexRightConfig.closedLoop.pid(
        Constants.ElevatorConstants.kP,
        Constants.ElevatorConstants.kI,
        Constants.ElevatorConstants.kD);

    SparkUtil.tryUntilOk(
        vortexLeft,
        5,
        () ->
            vortexLeft.configure(
                vortexLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    SparkUtil.tryUntilOk(
        vortexRight,
        5,
        () ->
            vortexRight.configure(
                vortexRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    pidController = vortexLeft.getClosedLoopController();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.motorAmps = new double[2];
    SparkUtil.ifOk(
        vortexLeft, vortexLeft::getOutputCurrent, (value) -> inputs.motorAmps[0] = value);
    SparkUtil.ifOk(
        vortexRight, vortexRight::getOutputCurrent, (value) -> inputs.motorAmps[1] = value);

    // inputs.motorVolts = new double[2];
    SparkUtil.ifOk(vortexLeft, leftEncoder::getPosition, (value) -> inputs.heightMeters = value);

    inputs.isOpenLoop = this.isOpenLoop;

    if (isOpenLoop) {
      this.vortexLeft.setVoltage(this.controlVoltage);
      this.vortexRight.setVoltage(this.controlVoltage);
    } else {
      pidController.setReference(this.goalMeters, ControlType.kPosition);
    }
  }

  @Override
  public void setPositionMeters(double positionMeters) {
    isOpenLoop = false;
    this.goalMeters = positionMeters;
  }

  public void setVoltage(double voltage) {
    isOpenLoop = true;
    this.controlVoltage = voltage;
    this.vortexLeft.setVoltage(voltage);
  }

  public void setZero() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public void disableSoftLimits(){
    vortexLeftConfig.softLimit
      .forwardSoftLimitEnabled(false)
      .reverseSoftLimitEnabled(false);
    SparkUtil.tryUntilOk(
        vortexLeft,
        5,
        () ->
            vortexLeft.configure(
                vortexLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void enableSoftLimits(){
    vortexLeftConfig.softLimit
      .forwardSoftLimitEnabled(Constants.ElevatorConstants.kElevatorLimitsEnabled)
      .reverseSoftLimitEnabled(Constants.ElevatorConstants.kElevatorLimitsEnabled);
    SparkUtil.tryUntilOk(
        vortexLeft,
        5,
        () ->
            vortexLeft.configure(
                vortexLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void setBrakeMode(boolean enable) {
    var brakeConfig = new SparkFlexConfig();
    brakeConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    SparkUtil.tryUntilOk(
        vortexLeft,
        5,
        () ->
            vortexLeft.configure(
                brakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
    SparkUtil.tryUntilOk(
        vortexLeft,
        5,
        () ->
            vortexLeft.configure(
                brakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }
}
