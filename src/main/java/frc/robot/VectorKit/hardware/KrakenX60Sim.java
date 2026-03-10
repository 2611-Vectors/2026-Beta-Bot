// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.VectorKit.hardware;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class KrakenX60Sim extends KrakenX60 {
    private final TalonFXSimState m_simState;
    private final Timer m_simTimer;

    private DCMotorSim m_sim;
    private DCMotor simMotor;
    private Boolean focMode = false;
    private double lastTime;

    public KrakenX60Sim(int ID) {
        super(ID);

        m_simTimer = new Timer();
        m_simState = kMotor.getSimState();
        initSimMotor();

        lastTime = m_simTimer.get();
    }

    private void initSimMotor() {
        simMotor = focMode ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1);
        m_sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(simMotor, 0.001, 1), simMotor);
    }

    @Override
    public void periodic() {
        super.periodic();
        m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        var motorVoltage = m_simState.getMotorVoltageMeasure();

        m_sim.setInputVoltage(motorVoltage.in(Volts));
        m_sim.update(m_simTimer.get() - lastTime);
        lastTime = m_simTimer.get();

        m_simState.setRawRotorPosition(m_sim.getAngularPosition());
        m_simState.setRotorVelocity(m_sim.getAngularVelocity());
    }
}
