// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.VectorKit.hardware;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;

/** Add your docs here. */
public class KrakenX44 extends KrakenX60 {
    public KrakenX44(int ID) {
        super(ID);
        logBaseDir = String.format("KrakenX44-%d", ID);
        if (RobotBase.isSimulation()) kSimState.setMotorType(TalonFXSimState.MotorType.KrakenX44);
    }

    @Override
    protected DCMotor getMotorSim() {
        return DCMotor.getKrakenX44(1);
    }
}
