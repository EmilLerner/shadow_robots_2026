#pragma once

namespace Control {
  void onTorque(float nm);   // call on each new torque sample
  void update();             // call every loop: applies outputs and buzz overlay
}
