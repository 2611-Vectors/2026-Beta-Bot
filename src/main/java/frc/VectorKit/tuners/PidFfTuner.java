// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.VectorKit.tuners;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Add your docs here. */
public class PidFfTuner {
    private final LoggedNetworkNumber kP, kI, kD, kS, kV;
    private double p, i, d, s, v;

    public PidFfTuner(String pathBase) {
        kP = new LoggedNetworkNumber(String.format("%s/pidFfTuner/P", pathBase), 0.0);
        kI = new LoggedNetworkNumber(String.format("%s/pidFfTuner/I", pathBase), 0.0);
        kD = new LoggedNetworkNumber(String.format("%s/pidFfTuner/D", pathBase), 0.0);
        kS = new LoggedNetworkNumber(String.format("%s/pidFfTuner/S", pathBase), 0.0);
        kV = new LoggedNetworkNumber(String.format("%s/pidFfTuner/V", pathBase), 0.0);
    }

    public PidFfTuner(
            String pathBase, double defaultP, double defaultI, double defaultD, double defaultS, double defaultV) {
        kP = new LoggedNetworkNumber(String.format("%s/pidFfTuner/P", pathBase), defaultP);
        kI = new LoggedNetworkNumber(String.format("%s/pidFfTuner/I", pathBase), defaultI);
        kD = new LoggedNetworkNumber(String.format("%s/pidFfTuner/D", pathBase), defaultD);
        kS = new LoggedNetworkNumber(String.format("%s/pidFfTuner/S", pathBase), defaultS);
        kV = new LoggedNetworkNumber(String.format("%s/pidFfTuner/V", pathBase), defaultV);
    }

    public Boolean updateCtreSlot(SlotConfigs slotConfigs) {
        Boolean updated = false;

        if (p != kP.get()) {
            updated = true;
            p = kP.get();
        }

        if (i != kI.get()) {
            updated = true;
            i = kI.get();
        }

        if (d != kD.get()) {
            updated = true;
            d = kD.get();
        }

        if (s != kS.get()) {
            updated = true;
            s = kS.get();
        }

        if (v != kV.get()) {
            updated = true;
            v = kV.get();
        }

        slotConfigs.withKP(p).withKI(i).withKD(d).withKS(s).withKV(v);

        return updated;
    }

    public Boolean updateCtreSlot(Slot0Configs slotConfigs) {
        Boolean updated = false;

        if (p != kP.get()) {
            updated = true;
            p = kP.get();
        }

        if (i != kI.get()) {
            updated = true;
            i = kI.get();
        }

        if (d != kD.get()) {
            updated = true;
            d = kD.get();
        }

        if (s != kS.get()) {
            updated = true;
            s = kS.get();
        }

        if (v != kV.get()) {
            updated = true;
            v = kV.get();
        }

        slotConfigs.withKP(p).withKI(i).withKD(d).withKS(s).withKV(v);

        return updated;
    }

    public Boolean updateCtreSlot(Slot1Configs slotConfigs) {
        Boolean updated = false;

        if (p != kP.get()) {
            updated = true;
            p = kP.get();
        }

        if (i != kI.get()) {
            updated = true;
            i = kI.get();
        }

        if (d != kD.get()) {
            updated = true;
            d = kD.get();
        }

        if (s != kS.get()) {
            updated = true;
            s = kS.get();
        }

        if (v != kV.get()) {
            updated = true;
            v = kV.get();
        }

        slotConfigs.withKP(p).withKI(i).withKD(d).withKS(s).withKV(v);

        return updated;
    }

    public Boolean updateCtreSlot(Slot2Configs slotConfigs) {
        Boolean updated = false;

        if (p != kP.get()) {
            updated = true;
            p = kP.get();
        }

        if (i != kI.get()) {
            updated = true;
            i = kI.get();
        }

        if (d != kD.get()) {
            updated = true;
            d = kD.get();
        }

        if (s != kS.get()) {
            updated = true;
            s = kS.get();
        }

        if (v != kV.get()) {
            updated = true;
            v = kV.get();
        }

        slotConfigs.withKP(p).withKI(i).withKD(d).withKS(s).withKV(v);

        return updated;
    }
}
