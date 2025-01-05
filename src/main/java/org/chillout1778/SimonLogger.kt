package org.chillout1778

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.networktables.DoubleArraySubscriber
import edu.wpi.first.networktables.DoubleArrayTopic
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.util.datalog.DoubleArrayLogEntry
import edu.wpi.first.util.datalog.StructLogEntry
import edu.wpi.first.wpilibj.DataLogManager
import org.chillout1778.subsystems.Swerve

class SimonLogger {
    init {
        DataLogManager.start()
        DataLogManager.logNetworkTables(false)
    }
    val log = DataLogManager.getLog()
    val odometryLog = StructLogEntry.create(log, "/odometry", Pose2d.struct)
    val fiducialsLog = DoubleArrayLogEntry(log, "/fiducials")
    val fiducialsSub = NetworkTableInstance.getDefault()
        .getTable("limelight")
        .getDoubleArrayTopic("rawfiducials")
        .subscribe(doubleArrayOf())
    fun periodic() {
        odometryLog.append(Swerve.odometry.poseMeters)
        fiducialsLog.append(fiducialsSub.get())
    }
}