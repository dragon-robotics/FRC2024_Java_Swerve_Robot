// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

  public final NetworkTable limelightNTable;

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
    limelightNTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  // Basic Targeting Data //
  public double getTv() {
    return limelightNTable.getEntry("tv").getDouble(0);
  }

  public double getTx() {
    return limelightNTable.getEntry("tx").getDouble(0);
  }

  public double getTy() {
    return limelightNTable.getEntry("ty").getDouble(0);
  }

  public double getTa() {
    return limelightNTable.getEntry("ta").getDouble(0);
  }

  public double getTl() {
    return limelightNTable.getEntry("tl").getDouble(0);
  }

  public double getCl() {
    return limelightNTable.getEntry("cl").getDouble(0);
  }

  public double getTshort() {
    return limelightNTable.getEntry("tshort").getDouble(0);
  }

  public double getTlong() {
    return limelightNTable.getEntry("tlong").getDouble(0);
  }

  public double getThor() {
    return limelightNTable.getEntry("thor").getDouble(0);
  }

  public double getTvert() {
    return limelightNTable.getEntry("tvert").getDouble(0);
  }

  public double getPipe() {
    return limelightNTable.getEntry("getpipe").getDouble(0);
  }

  public double getJson() {
    return limelightNTable.getEntry("json").getDouble(0);
  }

  public double getTclass() {
    return limelightNTable.getEntry("tclass").getDouble(0);
  }

  public double getTc() {
    return limelightNTable.getEntry("tc").getDouble(0);
  }

  // AprilTags and 3D Data //
  public double[] getBotPose(){
    return limelightNTable.getEntry("botpose").getDoubleArray(new double[6]);
  }

  public double[] getBotPoseWpiBlue(){
    return limelightNTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
  }
  
  public double[] getBotPoseWpiRed(){
    return limelightNTable.getEntry("botpose_wpired").getDoubleArray(new double[6]);
  }
  
  public double[] getCameraPoseTargetSpace(){
    return limelightNTable.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
  }
  
  public double[] getTargetPoseCameraSpace(){
    return limelightNTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
  }
  
  public double[] getTargetPoseRobotSpace(){
    return limelightNTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
  }
  
  public double[] getBotPoseTargetSpace(){
    return limelightNTable.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
  }
  
  public double[] getCameraPoseRobotSpace(){
    return limelightNTable.getEntry("camerapose_robotspace").getDoubleArray(new double[6]);
  }
  
  public double[] tid(){
    return limelightNTable.getEntry("tid").getDoubleArray(new double[6]);
  }

  // Camera Controls //
  public void setLedMode(double mode) {
    limelightNTable.getEntry("ledMode").setNumber(mode);
  }

  public void setCamMode(double mode) {
    limelightNTable.getEntry("camMode").setNumber(mode);
  }

  public void setPipeline(double pipeline) {
    limelightNTable.getEntry("pipeline").setNumber(pipeline);
  }

  public void setStream(double stream) {
    limelightNTable.getEntry("stream").setNumber(stream);
  }

  public void setSnapshot(double snapshot) {
    limelightNTable.getEntry("snapshot").setNumber(snapshot);
  }

  public void setCrop(double[] cropValues) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("crop").setDoubleArray(cropValues);
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
