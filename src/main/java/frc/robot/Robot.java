// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class Robot extends TimedRobot {
  private static final double kMinHeightMeters = 0.165;
  private static final double kMaxHeightMeters = 1.40;
  private static final double kDrumRadiusMeters = 0.02;
  private static final double kGearRatio = 10.0;
  private static final double kRotationsPerMeter =
      kGearRatio / (2.0 * Math.PI * kDrumRadiusMeters);

  private final TalonFX m_fx = new TalonFX(1, new CANBus("canivore"));
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
  private final NeutralOut m_brake = new NeutralOut();
  private final XboxController m_joystick = new XboxController(0);
  private final Mechanisms m_mechanism = new Mechanisms();

  private final ElevatorSim m_elevatorSim = new ElevatorSim(
      DCMotor.getKrakenX60Foc(1),
      kGearRatio,
      8.0,
      kDrumRadiusMeters,
      kMinHeightMeters,
      kMaxHeightMeters,
      true,
      kMinHeightMeters);

  private double m_targetHeightMeters = kMinHeightMeters;

  public Robot() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 24.0;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 1.0;
    configs.Slot0.kG = 0.35;
    configs.Voltage.withPeakForwardVoltage(Volts.of(8))
        .withPeakReverseVoltage(Volts.of(-8));
    configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(80))
        .withPeakReverseTorqueCurrent(Amps.of(-80));

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_fx.getConfigurator().apply(configs);
      if (status.isOK()) {
        break;
      }
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status);
    }
    m_fx.setPosition(kMinHeightMeters * kRotationsPerMeter);
  }

  @Override
  public void robotPeriodic() {
    m_mechanism.update(m_elevatorSim.getPositionMeters(), kMinHeightMeters);
  }

  @Override
  public void teleopPeriodic() {
    double joystick = MathUtil.applyDeadband(-m_joystick.getLeftY(), 0.08);
    m_targetHeightMeters = MathUtil.clamp(
        kMinHeightMeters + joystick * (kMaxHeightMeters - kMinHeightMeters),
        kMinHeightMeters,
        kMaxHeightMeters);

    m_fx.setControl(m_positionVoltage.withPosition(m_targetHeightMeters * kRotationsPerMeter));
  }

  @Override
  public void disabledInit() {
    m_fx.setControl(m_brake);
  }

  @Override
  public void simulationInit() {
    TalonFXSimState motorSim = m_fx.getSimState();
    motorSim.setSupplyVoltage(12.0);
    motorSim.setRawRotorPosition(kMinHeightMeters * kRotationsPerMeter);
  }

  @Override
  public void simulationPeriodic() {
    TalonFXSimState motorSim = m_fx.getSimState();
    m_elevatorSim.setInputVoltage(motorSim.getMotorVoltage());
    m_elevatorSim.update(0.02);

    motorSim.setRawRotorPosition(m_elevatorSim.getPositionMeters() * kRotationsPerMeter);
    motorSim.setRotorVelocity(m_elevatorSim.getVelocityMetersPerSecond() * kRotationsPerMeter);
    motorSim.setSupplyVoltage(12.0);
  }
}
