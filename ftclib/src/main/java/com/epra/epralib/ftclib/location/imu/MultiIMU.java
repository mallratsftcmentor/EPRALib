package com.epra.epralib.ftclib.location.imu;

import com.epra.epralib.ftclib.math.geometry.Angle;
import com.epra.epralib.ftclib.math.geometry.Geometry;
import com.epra.epralib.ftclib.storage.logdata.DataLogger;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

/// An object that can combine data from any number of [IMUWrapper]s.
///
/// @author Striker-909
/// @author jfishCoder
public class MultiIMU implements DataLogger {

    private Angle baseYaw = new Angle();
    private Angle basePitch = new Angle();
    private Angle baseRoll = new Angle();

    public enum Axis {
        YAW,
        PITCH,
        ROLL
    }

    ArrayList<IMUWrapper> imus = new ArrayList<>();

    private final String logPath;
    private final HashMap<String, Double> logData;
    private final Axis[] loggingTargets;

    /// An object that can combine data from any number of [IMUWrapper]s.
    ///
    /// @param imu An IMUWrapper
    /// @param imus Any number of additional IMUWrappers
    /// @param loggingTargets An array of the axes that this multiIMU should log rotational data on
    public MultiIMU(IMUWrapper imu, IMUWrapper[] imus, Axis[] loggingTargets) {
        this.imus.add(imu);
        this.imus.addAll(java.util.Arrays.asList(imus));

        this.logPath = "IMU.json";
        this.loggingTargets = loggingTargets;
        this.logData = new HashMap<>();
        recenter();
    }
    /// An object that can combine data from any number of [IMUWrapper]s.
    ///
    /// @param imus Any number of additional IMUWrappers
    /// @param loggingTargets An array of the axes that this multiIMU should log rotational data on
    public MultiIMU(IMUWrapper[] imus, Axis[] loggingTargets) {
        this.imus.addAll(java.util.Arrays.asList(imus));

        this.logPath = "IMU.json";
        this.loggingTargets = loggingTargets;
        this.logData = new HashMap<>();
        recenter();
    }

    /// A builder for a [MultiIMU].
    public static class Builder {
        private ArrayList<IMUWrapper> imus;
        private Angle initialYaw = new Angle();
        private Angle initialPitch = new Angle();
        private Angle initialRoll = new Angle();
        private ArrayList<Axis> loggingTargets;

        /// A builder for a [MultiIMU].
        ///
        /// Starts with default parameters:
        /// - Only one [IMUWrapper].
        /// - No logging targets.
        /// @param imu An IMUWrapper
        ///
        /// @see #imu(IMUWrapper...)
        /// @see #loggingTarget(Axis...)
        /// @see #build()
        public Builder(IMUWrapper imu) {
            imus = new ArrayList<>();
            imus.add(imu);
            loggingTargets = new ArrayList<>();
        }

        /// Adds any number of [IMUWrapper]s to the builder.
        /// @param imus Any number of IMUWrappers
        /// @return This builder
        public Builder imu(IMUWrapper... imus) {
            this.imus.addAll(java.util.Arrays.asList(imus));
            return this;
        }
        /// Sets the initial yaw value for IMUWrapper.
        /// @param initialYaw The initial yaw value
        /// @return This builder
        public Builder initialYaw(Angle initialYaw) {
            this.initialYaw = initialYaw;
            return this;
        }
        /// Sets the initial pitch value for IMUWrapper.
        /// @param initialPitch The initial pitch value
        /// @return This builder
        public Builder initialPitch(Angle initialPitch) {
            this.initialPitch = initialPitch;
            return this;
        }
        /// Sets the initial roll value for IMUWrapper.
        /// @param initialRoll The initial yaw value
        /// @return This builder
        public Builder initialRoll(Angle initialRoll) {
            this.initialRoll = initialRoll;
            return this;
        }
        /// Adds any number of [Axes][Axis] for the [MultiIMU] to log data from.
        /// @param axes Any number of logging targets
        /// @return This builder
        public Builder loggingTarget(Axis... axes) {
            this.loggingTargets.addAll(Arrays.asList(axes));
            return this;
        }
        /// Builds a [MultiIMU] from this builder.
        /// @return The multiIMU built from this builder
        public  MultiIMU build() {
            MultiIMU temp =  new MultiIMU(imus.toArray(imus.toArray(new IMUWrapper[0])), loggingTargets.toArray(new Axis[0]));
            temp.baseYaw = initialYaw;
            temp.basePitch = initialPitch;
            temp.baseRoll = initialRoll;
            return temp;
        }
    }

    /// Averages the [Axis#YAW] [Angle] from all [IMUWrapper]s.
    /// @return The average yaw angle
    public Angle getYaw() {
        Angle[] angle = new Angle[imus.size()];
        for (int i = 0; i < imus.size(); i++) {
            angle[i] = imus.get(i).getYaw();
        }
        return Geometry.subtract(Geometry.average(angle), baseYaw);
    }

    /// Averages the [Axis#PITCH] [Angle] from all [IMUWrapper]s.
    /// @return The average pitch angle
    public Angle getPitch() {
        Angle[] angle = new Angle[imus.size()];
        for (int i = 0; i < imus.size(); i++) {
            angle[i] = imus.get(i).getPitch();
        }
        return Geometry.subtract(Geometry.average(angle), basePitch);
    }

    /// Averages the [Axis#ROLL] [Angle] from all [IMUWrapper]s.
    /// @return The average roll angle
    public Angle getRoll() {
        Angle[] angle = new Angle[imus.size()];
        for (int i = 0; i < imus.size(); i++) {
            angle[i] = imus.get(i).getRoll();
        }
        return Geometry.subtract(Geometry.average(angle), baseRoll);
    }

    /// Averages the [Angle] around the specified [Axis] from all [IMUWrapper]s.
    /// @param axis An axis
    /// @return The average angle around the axis
    public Angle getAngle(Axis axis) {
        return switch (axis) {
            case YAW -> getYaw();
            case PITCH -> getPitch();
            case ROLL -> getRoll();
        };
    }

    /// Averages the [Axis#YAW] angular velocity from all [IMUWrapper]s,
    /// returns it as an [Angle].
    /// @return The average yaw angular velocity
    public Angle getYawVelocity() {
        Angle[] angle = new Angle[imus.size()];
        for (int i = 0; i < imus.size(); i++) {
            angle[i] = imus.get(i).getYawVelocity();
        }
        return Geometry.average(angle);
    }

    /// Averages the [Axis#PITCH] angular velocity from all [IMUWrapper]s,
    /// returns it as an [Angle].
    /// @return The average pitch angular velocity
    public Angle getPitchVelocity() {
        Angle[] angle = new Angle[imus.size()];
        for (int i = 0; i < imus.size(); i++) {
            angle[i] = imus.get(i).getPitchVelocity();
        }
        return Geometry.average(angle);
    }

    /// Averages the [Axis#ROLL] angular velocity from all [IMUWrapper]s,
    /// returns it as an [Angle].
    /// @return The average roll angular velocity
    public Angle getRollVelocity() {
        Angle[] angle = new Angle[imus.size()];
        for (int i = 0; i < imus.size(); i++) {
            angle[i] = imus.get(i).getRollVelocity();
        }
        return Geometry.average(angle);
    }

    /// Averages the angular velocity around the specified [Axis] from all [IMUWrapper]s,
    /// returns it as an [Angle].
    /// @param axis An axis
    /// @return The average angular velocity around the axis
    public Angle getAngularVelocity(Axis axis) {
        return switch (axis) {
            case YAW -> getYawVelocity();
            case PITCH -> getPitchVelocity();
            case ROLL -> getRollVelocity();
        };
    }

    /// Sets the current [Axis#YAW], [Axis#PITCH], and [Axis#ROLL] of all [IMUWrapper]s to be the new zero.
    public void recenter() {
        Angle[] yaw = new Angle[imus.size()];
        Angle[] pitch = new Angle[imus.size()];
        Angle[] roll = new Angle[imus.size()];
        for (int i = 0; i < imus.size(); i++) {
            yaw[i] = imus.get(i).getYaw();
            pitch[i] = imus.get(i).getPitch();
            roll[i] = imus.get(i).getRoll();
        }
        baseYaw = Geometry.average(yaw);
        basePitch = Geometry.average(pitch);
        baseRoll = Geometry.average(roll);
    }

    /// {@inheritDoc}
    /// @return The relative file path for the logs from this logger
    @Override
    public String getLogPath() { return logPath; }
    /// Updates the values for all [Axes][Axis] that are logging targets.
    /// @return `True`
    @Override
    public boolean updateLog() {
        for (Axis axis : loggingTargets) {
            logData.put(axis.toString().toLowerCase(), getAngle(axis).degree());
        }
        return true;
    }
    /// {@inheritDoc}
    /// @return A hash map with a data snapshot of this logger
    @Override
    public HashMap<String, Double> logData() { return logData; }
    /// {@inheritDoc}
    /// @return The unix time in milliseconds when the ping was received
    @Override
    public long ping() { return System.currentTimeMillis(); }
}