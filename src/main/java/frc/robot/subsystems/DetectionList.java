package frc.robot.subsystems;

import java.util.ArrayList;

class DetectionList extends ArrayList<Detection> {
  @Override
  public boolean add(Detection detec) {
    return super.add(detec);
  }

  @Override
  public Detection get(int index) {
    return super.get(index);
  }

  @Override
  public Detection remove(int index) {
    return super.remove(index);
  }
}
