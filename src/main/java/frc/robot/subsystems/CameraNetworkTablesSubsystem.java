// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.IntegerArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraNetworkTablesSubsystem extends SubsystemBase {
  /** Creates a new CameraNetworkTablesSubsystem. */
  private NetworkTableInstance ntinst;

  private NetworkTable table;
  int listenerHandle;


  private DoubleArrayEntry xEntry;
  private DoubleArrayEntry yEntry;
  private DoubleArrayEntry zEntry;
  private IntegerArrayEntry aprilTagIdEntry;
  public CameraNetworkTablesSubsystem() {ntinst = NetworkTableInstance.getDefault();
    ntinst.setServer("..."); //find out server // We are running network tables on the same computer as the robot code to avoid bandwidth issues during comp
    // ntinst.setServer("127.0.0.1");
    table = ntinst.getTable("..."); //find out key
    ntinst.removeListener(0);

    // button0Entry = table.getBooleanTopic("0").getEntry(false);
    // selectedProgramEntry = table.getStringTopic("SelectedProgramString2").getEntry("");

    getEntries();
  }

  private void getEntries(){
    xEntry = table.getDoubleArrayTopic("x").getEntry(null);
    xEntry = table.getDoubleArrayTopic("y").getEntry(null);
    xEntry = table.getDoubleArrayTopic("z").getEntry(null);
    aprilTagIdEntry = table.getIntegerArrayTopic("aprilTagId").getEntry(null);
  }

   public long[] getAprilTagEntry(){
    long[] aprilTagIds = aprilTagIdEntry.get();
    try{
      // SmartDashboard.putNumberArray("Streamdeck_elevState", aprilTagIds);
    }
    catch(Exception e){
      
    }
    return aprilTagIds;
  }

  // public String getAutoEntry(){
  //   String autoVal = autoEntry.get();

  //   try{
  //     SmartDashboard.putString("StreamDeck_autoEntry", autoVal);
  //   }
  //   catch(Exception e){
      
  //   }
  //   return autoVal;
  // }

  // public void clear(){
  //   for(String index: table.getKeys()){
  //     NetworkTableEntry entry = table.getEntry(index);
  //     entry.clearPersistent();
  //     entry.setDefaultValue(false);
  //   }
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
