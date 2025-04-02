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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private double lp, li, ld, lf;

  public VortexElevatorIO() {
    SmartDashboard.putNumber("elev/p", Constants.ElevatorConstants.kP);
    SmartDashboard.putNumber("elev/i", Constants.ElevatorConstants.kI);
    SmartDashboard.putNumber("elev/d", Constants.ElevatorConstants.kD);
    SmartDashboard.putNumber("elev/f", 0);
    lp = Constants.ElevatorConstants.kP;
    ld = Constants.ElevatorConstants.kD;
    li = Constants.ElevatorConstants.kI;
    lf = 0;
    vortexLeftConfig = new SparkFlexConfig();
    vortexLeftConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .voltageCompensation(12)
        .inverted(true)
        .encoder
        .positionConversionFactor(Constants.ElevatorConstants.conversionFactor);
    vortexLeftConfig.closedLoop.pidf(
        Constants.ElevatorConstants.kP,
        Constants.ElevatorConstants.kI,
        Constants.ElevatorConstants.kD,
        Constants.ElevatorConstants.kF);

    vortexRightConfig = new SparkFlexConfig();
    vortexRightConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .voltageCompensation(12)
        .follow(vortexLeft, true)
        .encoder
        .positionConversionFactor(Constants.ElevatorConstants.conversionFactor);
    vortexRightConfig.closedLoop.pidf(
        Constants.ElevatorConstants.kP,
        Constants.ElevatorConstants.kI,
        Constants.ElevatorConstants.kD,
        Constants.ElevatorConstants.kF);

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
    SmartDashboard.putNumber("elevator output", vortexLeft.getAppliedOutput());

    // double np = SmartDashboard.getNumber("elev/p", lp);
    // double ni = SmartDashboard.getNumber("elev/i", li);
    // double nd = SmartDashboard.getNumber("elev/d", ld);
    // double nf = SmartDashboard.getNumber("elev/f", lf);

    // if (np != lp || ni != lp || nd != ld || nf != lf){

    // }
    // lp = np;
    // li = ni;
    // ld = nd;
    // lf = nf;
    inputs.motorAmps = new double[2];
    SparkUtil.ifOk(
        vortexLeft, vortexLeft::getOutputCurrent, (value) -> inputs.motorAmps[0] = value);
    SparkUtil.ifOk(
        vortexRight, vortexRight::getOutputCurrent, (value) -> inputs.motorAmps[1] = value);

    SparkUtil.ifOk(
        vortexLeft, vortexLeft::getMotorTemperature, (value) -> inputs.leftMotorTemp = value);
    SparkUtil.ifOk(
        vortexRight, vortexRight::getMotorTemperature, (value) -> inputs.rightMotorTemp = value);
    // inputs.motorVolts = new double[2];
    SparkUtil.ifOk(
        vortexLeft, leftEncoder::getPosition, (value) -> inputs.heightMeters = value / 100.0);

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
    // change the setpoint from meters to centimeters
    this.goalMeters = positionMeters * 100.0;
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
