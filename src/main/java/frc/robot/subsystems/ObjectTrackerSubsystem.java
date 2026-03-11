package frc.robot.subsystems;

import com.google.gson.Gson;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.models.VisionObject;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

/**
 * TODO: if we get ObjectTrackerSubsystem working well, there is potential to use
 * SwerveDriveOdometry.resetPosition() based off of the vision measurements. If this is done,
 * possibly could use path-planner to create paths for us on the fly once the robot knows where it
 * is on the field. This could also be used to reset the current position of the robot during a long
 * auto with multiple segments. Note that we would need to ensure that the path could be calculated
 * within one periodic loop and not bog down the code and create clock over-runs.
 */
public class ObjectTrackerSubsystem extends SubsystemBase {
  // // TODO: Harmonize Detection and VisionObject and refactor code
  // public class Detection{
  //     public String objectLabel;
  //     public double x;
  //     public double y;
  //     public double z;
  //     public double confidence;
  //     public double xa;
  //     public double ya;
  //     public double za;
  // }
  // class DetectionList extends ArrayList<Detection> {
  //     @Override
  //     public boolean add(Detection detec) {
  //         return super.add(detec);
  //     }
  //     @Override
  //     public Detection get(int index) {
  //         return super.get(index);
  //     }
  //     @Override
  //     public Detection remove(int index) {
  //         return super.remove(index);
  //     }
  // }

  NetworkTable monsterVision;
  public VisionObject[] foundObjects;
  private String jsonString;
  private String source;
  private Gson gson = new Gson();

  public double visionZ;
  public double visionX;
  public double visionY;
  public double visionYa;

  private final double CAMERA_PITCH_FRONT = 0; // should be 23
  private final double CAMERA_PITCH_BACK = 0;
  private double m_cameraPitch = Constants.CAMERA_TILT;

  // rotation matrix
  private double cameraTilt = Constants.CAMERA_TILT;
  private double[] cameraOffset =
      Constants.CAMERA_OFFSET; // goes {x, y} // In inches // TODO: figure this offset

  private double cosTheta;

  public DetectionList yoloObjects;
  public DetectionList aprilTags;

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public ObjectTrackerSubsystem(String source) {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    this.source = source;
    monsterVision = inst.getTable("MonsterVision");
    jsonString = "";

    // if (source == "back") { // Network Table that MonsterVision writes to
    //     cameraOffset = Constants.VISION_BALLOON_CAM_OFFSET;
    //     m_cameraPitch = CAMERA_PITCH_BACK;
    // }
    // else if(source == "front"){ //Network Table that MonsterVision writes to
    //     cameraOffset=Constants.VISION_TOTE_CAM_OFFSET;
    //     m_cameraPitch = CAMERA_PITCH_FRONT;
    // }
    cosTheta = Math.cos(Math.toRadians(cameraTilt));

    yoloObjects = new DetectionList();
    aprilTags = new DetectionList();
  }

  public void data() {
    NetworkTableEntry entry = monsterVision.getEntry("ObjectTracker-" + source);
    if (entry == null) {
      return;
    }
    // default to an empty list of detections if nothing is found:
    jsonString = entry.getString("[]");
    try {
      // TODO: call updateDetections with detectionsString = jsonString to populate yoloObjects and
      // aprilTags
      updateDetections(jsonString, gson);
      // use the getClosestAprilTag() to get the detection for the closest april tag
      // Use smart dashboarf to display the x, y, z and ya values

      SmartDashboard.putNumber("VisionX", getNearestAprilTagDetection().x);
      SmartDashboard.putNumber("VisionY", getNearestAprilTagDetection().y);
      SmartDashboard.putNumber("VisionZ", getNearestAprilTagDetection().z);
      SmartDashboard.putNumber("VisionYa", getNearestAprilTagDetection().ya);

      // System.out.println("x: "+ getNearestAprilTagDetection().x + ", y: "+
      // getNearestAprilTagDetection().y + ", z: " + getNearestAprilTagDetection().z + ", ya: "+
      // getNearestAprilTagDetection().ya);

      // lets just not use these at all...
      // visionZ = getNearestAprilTagDetection().z;
      // visionX =  getNearestAprilTagDetection().x;
      // visionY = getNearestAprilTagDetection().y;
      // visionYa =  getNearestAprilTagDetection().ya;

      // visionZ = getSpecificAprilTag(Robot.m_toteChooser.getSelected()).z;
      // visionX = getSpecificAprilTag(Robot.m_toteChooser.getSelected()).x;
      // visionY = getSpecificAprilTag(Robot.m_toteChooser.getSelected()).y;
      // visionYa = getSpecificAprilTag(Robot.m_toteChooser.getSelected()).ya;

      // String fpsString = monsterVision.getEntry("ObjectTracker-fps").getString("").substring(5);
      // double fps = Double.valueOf(fpsString);
      // SmartDashboard.putNumber("CameraFPS", fps);
    } catch (Exception e) {
      // System.out.println(e);
    }

    return;
  }

  // GET VISION DATA FROM NEAREST TAG
  public double getVisionX() {
    try {
      visionX = getNearestAprilTagDetection().x;
      return visionX;
    } catch (Exception e) {
      return 0;
    }
  }

  public double getVisionY() {
    try {
      visionY = getNearestAprilTagDetection().y;

      return visionY;
    } catch (Exception e) {
      return 0;
    }
  }

  public double getVisionZ() {
    try {
      visionZ = getNearestAprilTagDetection().z;

      return visionZ;
    } catch (Exception e) {
      return 0;
    }
  }

  public double getVisionYa() {
    try {
      visionYa = getNearestAprilTagDetection().ya;

      return visionYa;
    } catch (Exception e) {
      return 0;
    }
  }

  // GET VISION DATA FROM SPECIFIC TAG
  public double getVisionX(int tagId) {
    try {
      return getSpecificAprilTag(tagId).x;
    } catch (Exception e) {
      return 0;
    }
  }

  public double getVisionY(int tagId) {
    try {
      return getSpecificAprilTag(tagId).y;
    } catch (Exception e) {
      return 0;
    }
  }

  public double getVisionZ(int tagId) {
    try {
      return getSpecificAprilTag(tagId).z;
    } catch (Exception e) {
      return 0;
    }
  }

  public double getVisionYa(int tagId) {
    try {
      return getSpecificAprilTag(tagId).ya;
    } catch (Exception e) {
      return 0;
    }
  }

  // GET VISION DATA FROM MULTIPLE TAGS
  public double getVisionX(int[] tagIds) {
    for (int tagId : tagIds) {
      try {
        return getVisionX(tagId);
      } catch (Exception e) {
        continue;
      }
    }
    return 0;
  }

  public double getVisionY(int[] tagIds) {
    for (int tagId : tagIds) {
      try {
        return getVisionY(tagId);
      } catch (Exception e) {
        continue;
      }
    }
    return 0;
  }

  public double getVisionZ(int[] tagIds) {
    for (int tagId : tagIds) {
      try {
        return getVisionZ(tagId);
      } catch (Exception e) {
        continue;
      }
    }
    return 0;
  }

  public double getVisionYa(int[] tagIds) {
    for (int tagId : tagIds) {
      try {
        return getVisionYa(tagId);
      } catch (Exception e) {
        continue;
      }
    }
    return 0;
  }

  /*
   * Gets the vector from camera to target point based on apriltag
   */
  public Pose2d getDistVector(double xPrime, double zPrime, double finalYa, int tagId) {
    Detection detection = null;
    data();
    for (int i = 0; i < 100; i++) { // TODO: do we still want to keep for loop?
      try {
        detection = getSpecificAprilTag(tagId);
        break;
      } catch (Exception e) {
        System.out.println("Failed vision attempt " + i);
      }
    }
    if (detection == null) {
      SmartDashboard.putBoolean("ableToSeeAT", false);
      return new Pose2d(0, 0, new Rotation2d(0));
    }

    SmartDashboard.putBoolean("ableToSeeAT", true);

    // detection = m_ots.

    // double visionYa = -detection.ya;

    double visionYa = Math.atan(detection.z / (detection.x + 0.00001));
    double x_vt =
        xPrime * Math.cos(Math.toRadians(visionYa)) - zPrime * Math.sin(Math.toRadians(visionYa));
    double z_vt =
        xPrime * Math.sin(Math.toRadians(visionYa)) + zPrime * Math.cos(Math.toRadians(visionYa));
    // Should be offset variables and change based of camera location relative to the center of the
    // robot
    double deltaCamX = -(detection.x + x_vt);
    double deltaCamY = -(detection.z + z_vt);

    double finalAngle = visionYa + finalYa;

    return new Pose2d(
        Units.inchesToMeters(deltaCamX),
        Units.inchesToMeters(deltaCamY),
        new Rotation2d(Units.degreesToRadians(finalAngle)));
  }

  public Pose2d visionAutoData(double xPrime, double zPrime, double finalYa, int tagId) {
    Detection detection = null;
    for (int i = 0; i < 100; i++) { // TODO: do we still want to keep for loop?
      try {
        detection = getSpecificAprilTag(tagId);
        break;
      } catch (Exception e) {
        System.out.println("Failed vision attempt " + i);
      }
    }
    if (detection == null) {
      SmartDashboard.putBoolean("ableToSeeAT", false);
      return null;
    }

    SmartDashboard.putBoolean("ableToSeeAT", true);

    // detection = m_ots.

    double visionYa = -detection.ya;
    double x_vt =
        xPrime * Math.cos(Math.toRadians(visionYa)) - zPrime * Math.sin(Math.toRadians(visionYa));
    double z_vt =
        xPrime * Math.sin(Math.toRadians(visionYa)) + zPrime * Math.cos(Math.toRadians(visionYa));
    // Should be offset variables and change based of camera location relative to the center of the
    // robot
    double deltaRobotX = -(detection.x + x_vt - Constants.CAM_X_OFFSET);
    double deltaRobotY = -(detection.z + z_vt - Constants.CAM_Y_OFFSET);

    // double deltaRobotX = -(detection.x + x_vt - Constants.VISION_TOTE_CAM_OFFSET[0]);
    // double deltaRobotY = -(detection.z + z_vt - Constants.VISION_TOTE_CAM_OFFSET[1]);

    double botRadians = 0; // Units.degreesToRadians(m_dts.getPose().getRotation().getDegrees());
    double deltaFieldX = deltaRobotX * Math.cos(botRadians) - deltaRobotY * Math.sin(botRadians);
    double deltaFieldY = deltaRobotX * Math.sin(botRadians) + deltaRobotY * Math.cos(botRadians);
    double finalAngle = visionYa + finalYa + Units.radiansToDegrees(botRadians);

    return new Pose2d(
        Units.inchesToMeters(deltaFieldX),
        Units.inchesToMeters(deltaFieldY),
        new Rotation2d(Units.degreesToRadians(finalAngle)));
  }

  public Detection getAprilTagDetections(int[] tagIds) {
    Detection[] detections = new Detection[tagIds.length];
    for (int i = 0; i < tagIds.length; i++) {
      try {
        detections[i] = getSpecificAprilTag(tagIds[i]);
        return detections[i];
      } catch (Exception e) {
        detections[i] = null;
      }
    }
    return null;
  }

  public Detection[] getAllAprilTagDetections(int[] tagIds) {
    Detection[] detections = new Detection[tagIds.length];
    for (int i = 0; i < tagIds.length; i++) {
      try {
        detections[i] = getSpecificAprilTag(tagIds[i]);
      } catch (Exception e) {
        detections[i] = null;
      }
    }
    return detections;
  }

  // public Detection getNearestAprilTagDetection(int[] tagIds){
  //     Detection[] detections = getAprilTagDetections(tagIds);
  //     double minimum = Integer.MAX_VALUE;
  //     int minIndex = -1;

  //     for(int i =0; i<detections.length; i++){
  //         if (detections[i] != null && detections[i].z < minimum){
  //             minimum = detections[i].z;
  //             minIndex = i;
  //         }
  //     }

  //     if (minIndex == -1){
  //         return null;

  //     }

  //     return detections[minIndex];
  // }

  // private void applyRotationTranslationMatrix() {
  //     // sets reference to be the CENTER of the robot

  //     for (int i = 0; i < foundObjects.length; i++) {
  //         double x = foundObjects[i].x;
  //         double y = foundObjects[i].y;
  //         double z = foundObjects[i].z;

  //         // rotation + translation
  //         foundObjects[i].x = x + cameraOffset[0];
  //         foundObjects[i].y = y * cosTheta - z * sinTheta + cameraOffset[1];
  //         foundObjects[i].z = y * sinTheta + z * cosTheta + cameraOffset[2];
  //     }
  // }

  public double applyPitchCorrection(double pitchDegrees, double y, double z) {
    // Corrects for a positive pitch up camera angle
    // In camera coordiantes y is negative from the center of the camera going up
    double alpha =
        Math.atan(
            (-y) / z); // angle in camera coordinate system from center of camera to detected object
    // (fraction of the field view)
    SmartDashboard.putNumber("applyPitchCorrection.alpha", alpha);
    // double adjustedZ = (z * Math.cos(Math.toRadians(pitchDegrees) + alpha)) / Math.cos(alpha);
    double hyp = Math.hypot(z, y);
    double adjustedZ = hyp * Math.cos(Math.toRadians(pitchDegrees) + alpha);
    SmartDashboard.putNumber("applyPitchCorrection.adjustedZ", adjustedZ);
    return adjustedZ;
  }

  public String getObjectsJson() {
    return jsonString;
  }

  // private NetworkTableEntry getEntry(Integer index, String subkey) {
  //     try {
  //         NetworkTable table = monsterVision.getSubTable(index.toString());
  //         NetworkTableEntry entry = table.getEntry(subkey);
  //         return entry;
  //     }
  //     catch (Exception e){
  //         return null;
  //     }
  // }

  public VisionObject getClosestObject(String objectLabel) {

    VisionObject[] objects = getObjectsOfType(objectLabel);
    if (objects == null || objects.length == 0) {
      return null;
    }
    return objects[0];
  }

  public VisionObject getClosestObject() {
    VisionObject[] objects = getObjects(0.5);
    if (objects == null || objects.length == 0) {
      return null;
    }
    return objects[0];
  }

  public VisionObject getSecondClosestObject(String objectLabel) {
    VisionObject[] objects = getObjectsOfType(objectLabel);
    if (objects == null || objects.length == 0) {
      return null;
    }
    return objects[1];
  }

  /** Returns closest AprilTag */
  public VisionObject getClosestAprilTag() {
    VisionObject object = getClosestObject("tag");
    if (object == null) {
      return null;
    }
    return object;
  }

  public VisionObject getSpecificAprilTagVisionObject(int id) {
    String objectLabel = "tag16h5: " + id;
    VisionObject[] objects = getObjectsOfType(objectLabel);
    if (objects == null || objects.length == 0) {
      return null;
    }
    return objects[0];
  }

  /**
   * Returns whether closest cone/cube to the gripper if close enough to pick up
   *
   * @param isCube TRUE cube, FALSE cone
   */
  public boolean isGripperCloseEnough(boolean isCube) {
    // this target is the target y value when the object moves between the claws for pick up
    double targetY =
        isCube
            ? 5
            : 2; // TODO: figure this y position out (somehting <0 bc its below the cneter of the
    // FOV)
    double actualY = 0; // TODO: get current y of the object

    return actualY < targetY; // TODO may want to change min based on whether it's a cube or cone
  }

  public int numberOfObjects() {
    return foundObjects.length;
  }

  public VisionObject[] getObjects(double minimumConfidence) {

    if (foundObjects == null || foundObjects.length == 0) return null;

    List<VisionObject> filteredResult =
        Arrays.asList(foundObjects).stream()
            .filter(vo -> vo.confidence >= minimumConfidence)
            .collect(Collectors.toList());

    VisionObject filteredArray[] = new VisionObject[filteredResult.size()];
    return filteredResult.toArray(filteredArray);
  }

  public VisionObject[] getObjectsOfType(String objectLabel) {
    if (foundObjects == null || foundObjects.length == 0) return null;
    List<VisionObject> filteredResult =
        Arrays.asList(foundObjects).stream()
            .filter(
                vo ->
                    vo.objectLabel.contains(objectLabel)
                        && (objectLabel.contains("tag")
                            || vo.confidence
                                > .40)) // Uses .contains because vo.ObjectLabel has ID, ObjectLabel
            // does not
            .collect(Collectors.toList());

    VisionObject filteredArray[] = new VisionObject[filteredResult.size()];
    return filteredResult.toArray(filteredArray);
  }

  public void saveVisionSnapshot(String fileName) throws IOException {
    data();
    Gson gson = new Gson();
    String str = gson.toJson(foundObjects);
    BufferedWriter writer = new BufferedWriter(new FileWriter(fileName));
    writer.write(str);

    writer.close();
  }

  public VisionObject[] loadVisionSnapshot(String fileName) throws IOException {
    Path filePath = Path.of(fileName);
    Gson gson = new Gson();

    String json = Files.readString(filePath);
    VisionObject[] snapShotObjects = gson.fromJson(json, VisionObject[].class);

    return snapShotObjects;
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  // ================================================================================
  // Functions from MyClass from JDoodle

  public Detection getNearestAprilTagDetection() {
    if (aprilTags.size() > 0) {
      return aprilTags.get(0);
    }
    return null;
  }

  public int getNearestAprilTag() {
    if (aprilTags.size() > 0) {
      return Integer.parseInt(getNearestAprilTagDetection().objectLabel.substring(10));
    }
    return -1;
  }

  public Pose2d getNearestAprilTagDistShooter() {

    int nearestTag = getNearestAprilTag();
    if (nearestTag != -1) {
      if (nearestTag == 21
          || nearestTag == 26
          || nearestTag == 18
          || nearestTag == 10
          || nearestTag == 5
          || nearestTag == 2) {
        return getDistVector(
            0,
            Units.metersToInches(0.6), // -/+   .6m
            0,
            nearestTag);
      } else if (nearestTag == 25 || nearestTag == 27 || nearestTag == 9 || nearestTag == 11) {
        return getDistVector(
            Units.metersToInches(0.35),
            Units.metersToInches(0.6), // -/+   .6m
            0,
            nearestTag);
      } else if (nearestTag == 24 || nearestTag == 8) {
        return getDistVector(
            Units.metersToInches(-0.35),
            Units.metersToInches(0.6), // -/+   .6m
            0,
            nearestTag);
      }
    }
    return new Pose2d(0, 0, new Rotation2d(0));
  }

  public Pose2d getNearestAprilTagDistTurret() {
    int nearestTag = getNearestAprilTag();
    if (nearestTag != -1) {
      if (nearestTag == 21
          || nearestTag == 26
          || nearestTag == 18
          || nearestTag == 10
          || nearestTag == 5
          || nearestTag == 2) {
        return getDistVector(
            0,
            Units.metersToInches(0.8), // -/+   .6m
            0,
            nearestTag);
      } else if (nearestTag == 25 || nearestTag == 27 || nearestTag == 9 || nearestTag == 11) {
        return getDistVector(
            Units.metersToInches(0.35),
            Units.metersToInches(0.8), // -/+   .6m
            0,
            nearestTag);
      } else if (nearestTag == 24 || nearestTag == 8) {
        return getDistVector(
            Units.metersToInches(-0.35),
            Units.metersToInches(0.8), // -/+   .6m
            0,
            nearestTag);
      }
    }
    return new Pose2d(0, 0, new Rotation2d(0));
  }

  public Detection getSpecificAprilTag(int id) {
    Detection currentAprilTag;
    data();
    for (int i = 0; i < aprilTags.size(); i++) {
      currentAprilTag = aprilTags.get(i);
      // The .substring(10) is for this specific aprilTag family which is "tag36h11: "
      if (currentAprilTag.objectLabel.substring(10).equals("" + id)) {
        SmartDashboard.putString(currentAprilTag.objectLabel, ("get specific apriltag " + id));
        return currentAprilTag;
      }
    }
    return null;
  }

  public Detection getNearestAprilTagFromList(int[] ids) {
    Detection currentAprilTag;
    data();
    // the april tags or ordered from closest to furthest away...
    // so we go through and check if each one is in the list.  if it is,
    // then the first one we found is the one we want.
    for (int i = 0; i < aprilTags.size(); i++) {
      currentAprilTag = aprilTags.get(i);
      for (int j = 0; j < ids.length; j++) {
        // The .substring(10) is for this specific aprilTag family which is "tag36h11: "
        if (currentAprilTag.objectLabel.substring(10).equals("" + ids[j])) {
          SmartDashboard.putString(
              currentAprilTag.objectLabel, ("get specific apriltag " + ids[j]));
          return currentAprilTag;
        }
      }
    }
    return null;
  }

  public Detection[] getNearestAprilTagsDetection(int count) {
    Detection[] detections = new Detection[count];
    if (count <= aprilTags.size()) {
      for (int i = 0; i < count; i++) {
        detections[i] = aprilTags.get(i);
      }
      return detections;
    } else {
      return null;
    }
  }

  public Detection getNearestYoloDetection() {
    if (yoloObjects.size() > 0) {
      return yoloObjects.get(0);
    }
    return null;
  }

  public Detection[] getNearestYoloDetections(int count) {
    Detection[] detections = new Detection[count];
    if (count <= yoloObjects.size()) {
      for (int i = 0; i < count; i++) {
        detections[i] = yoloObjects.get(i);
      }
      return detections;
    } else {
      return null;
    }
  }

  // // Yaw difference is from the frame of reference of rotation
  // // Get Y from the rotaion object of detection and translate to z rotation of robot spo we can
  // directly feed into command to turn robot
  // // Confirm that we are turning in correct direction
  // public double getYawDifferenceFromDetectionRotation(Detection detection) {
  //     // The whole point of this functions is so that we know which direction to turn the robot
  //     return 0.0;
  // }
  // public double getDistanceFromDetection(Detection detection) {
  //     return detection.z;
  // }
  // public double getXFieldDistanceFromDetection(Detection detection) {
  //     // x not
  //     return 0.0;
  // }
  public double getThetaYZField(Detection detection) {
    double camX = detection.x;
    double camZ = detection.z;
    double yCamAngle = detection.ya;
    double thetaYZ = Math.tanh(camZ / camX);
    double thetaYZField = 90.0 - yCamAngle - thetaYZ;

    return thetaYZField;
  }

  public double getYFieldAprilFromDetection(Detection detection) {
    double yField;

    double radius = getRadius(detection);
    double thetaYZField = getThetaYZField(detection);

    yField = Math.cos(thetaYZField) * radius;

    return yField;
  }

  public double getXFieldAprilFromDetection(Detection detection) {
    double xField;

    double radius = getRadius(detection);
    double thetaYZField = getThetaYZField(detection);

    xField = Math.sin(thetaYZField) * radius;

    return xField;
  }

  public double getRadius(Detection detection) {
    double camX = detection.x;
    double camZ = detection.z;

    double radius = Math.sqrt(Math.pow(camX, 2) + Math.pow(camZ, 2));
    return radius;
  }

  //  // Iterate through the JSON array and convert each element to a Detection object
  // import org.json.JSONArray;
  // import org.json.JSONObject;
  // Parse the JSON string
  // JSONArray jsonArray = new JSONArray(jsonString);

  // // Create an ArrayList to hold the Detection objects
  // ArrayList<Detection> detections = new ArrayList<>();
  //  for (int i = 0; i < jsonArray.length(); i++) {
  //     JSONObject jsonObject = jsonArray.getJSONObject(i);
  //     // Extract values from the JSON object
  //     String objectLabel = jsonObject.getString("objectLabel");
  //     double x = jsonObject.getDouble("x");
  //     double y = jsonObject.getDouble("y");
  //     double z = jsonObject.getDouble("z");
  //     double confidence = jsonObject.getDouble("confidence");
  //     double xa = jsonObject.getDouble("xa");
  //     double ya = jsonObject.getDouble("ya");
  //     double za = jsonObject.getDouble("za");

  //     // Create a new Detection object and add it to the ArrayList
  //     Detection detection = new Detection(objectLabel, x, y, z, confidence, xa, ya, za);
  //     detections.add(detection);
  // }

  public Detection applyOffset(Detection detectionObject) {
    Detection offsetedDetection = detectionObject;
    offsetedDetection.z = cosTheta * offsetedDetection.z;
    offsetedDetection.z = offsetedDetection.z + cameraOffset[1]; // TODO: Figure out these signs
    offsetedDetection.x = offsetedDetection.x - cameraOffset[0];

    return offsetedDetection;
  }

  public void updateDetections(String detectionsString, Gson gson) {
    // System.out.println(detectionsString);
    DetectionList gsonOut = gson.fromJson(detectionsString, DetectionList.class);

    // Initialy grab fps from gsonOut, only update april tags and Yolo objects only if fps is above
    // 25
    String fpsString = monsterVision.getEntry("ObjectTracker-fps").getString("").substring(5);
    double fps = Double.valueOf(fpsString);

    // if (fps<25) {
    //     // If the frames per second is less than 25 don't do the update
    //     // TODO: debug why there is intermittent frame rate
    //     return ;
    // }

    SmartDashboard.putNumber("CameraFPS", fps);
    aprilTags.clear();
    yoloObjects.clear();

    // Seperate out the detections with rotation
    for (int i = 0; i < gsonOut.size(); i++) { // Maybe change later
      Detection detectionObject = (Detection) gsonOut.get(i);
      SmartDashboard.putNumber("updateDetections: raw z", detectionObject.z);
      detectionObject.z = applyPitchCorrection(m_cameraPitch, detectionObject.y, detectionObject.z);
      // detectionObject = applyOffset(detectionObject);
      SmartDashboard.putNumber("updateDetections.detectionObject.z", detectionObject.z);
      if (detectionObject.objectLabel.substring(0, 3).equals("tag")) {

        aprilTags.add(detectionObject);
        // aprilTags.add(adjustCamOffset(gsonOut.get(i)));
        // System.out.println("UpdateDetections(: found apriltag");
      } else {
        yoloObjects.add(detectionObject);
        // yoloObjects.add(adjustCamOffset(gsonOut.get(i)));
        // System.out.println("yolo object");
      }
    }
    try {
      if (yoloObjects.size() > 0) {
        SmartDashboard.putBoolean("AlgaeVisible", true);
      } else {
        SmartDashboard.putBoolean("AlgaeVisible", false);
      }
    } catch (Exception e) {
    }
  }

  // TODO: not working yet
  private Detection adjustCamOffset(Detection detection1) {
    Detection detection = detection1;
    if (source == "Balloon") { // TODO: figure out these source names from vision code
      detection.x += cameraOffset[0];
      detection.z += cameraOffset[1];
    } else if (source == "Eclipse") {
      detection.x -= cameraOffset[0];
      detection.z = detection.z + cameraOffset[1];
    }

    return detection;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    data();
  }
}

/*
 * To switch from MV4.5 to MV4.6 the steps are:
 * nano runCamera
 * then change the line with "cd" on it to have MV4.6 in the name
 * then cd MV4.6
 * then sudo cp ./models/MV46.json /boot/nn.json
 * then sudo nano /boot/nn.json
 * inside the nn_config part add (exactly including the comma): "blob": "MV46.blob",
 * then done?
 *
 * To switch back:
 * nano runCamera
 * then change the line with "cd" on it to have MV4.5 in the name
 * then cd MV4.5
 * then sudo cp ./models/2024.json /boot/nn.json
 * then sudo nano /boot/nn.json
 */
