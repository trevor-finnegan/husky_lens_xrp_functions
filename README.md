# **XRP Vision-Guided Robotics Project**

Real-Time Tag Navigation, Object Ramming, and Multi-Color Visual Tracking Using HuskyLens + XRP Robot

## **Overview**

This project integrates the **XRP educational robot** and the **HuskyLens AI vision module** to create a real-time, vision-guided robotic system.
The robot is capable of:

* **Tag-based autonomous navigation**
* **Object alignment and knock-over behavior**
* **Continuous visual tracking using multi-color ensembles (IDs 1–3)**
* **Distance-maintaining control using bounding-box area**
* **Closed-loop control via XRPLib over differential drive**

All perception runs on the HuskyLens (over I2C/Qwiic), while all motion control runs on the XRP.
Python scripts implement different behaviors using the same unified architecture.

---

## **Hardware Components**

### **XRP Robot**

* Dual DC motors for differential drive
* On-board Raspberry Pi–based controller
* XRPLib for high-level drivetrain and sensor access
* Optional servo mount for camera elevation

### **HuskyLens Vision Module**

* Runs lightweight CNN-based models for:

  * Tag Recognition
  * Object Recognition
  * Object Tracking
  * Color Recognition
* Provides bounding boxes (`xCenter`, `yCenter`, `width`, `height`, `id`)
* Communicates with the robot via I2C (Qwiic)

### Image of Robot:

[<img src="photo\xrp+husky.png" width="375"/>](photo\xrp+husky.png)

### **System Architecture**

1. HuskyLens performs real-time visual detection and classification.
2. Python scripts retrieve all “objects of interest” over I2C.
3. A custom state machine maps visual inputs to robot motion using XRPLib.
4. The robot executes navigation, tracking, or alignment behaviors depending on the module.

---

## **Repository Structure**

### **tag_recognition.py**

**Goal:** Sign-following and scripted motion based on recognized tag IDs.

**Behavior:**

* HuskyLens runs in **Tag Recognition** mode.
* For each detected ID:

  * **IDs 1–4:** Robot performs a fixed-angle turn.
  * **IDs 5–8:** Robot drives a fixed linear distance.
* Implements a simple perception → command loop for directional navigation.

---

### **track_object.py**

**Goal:** Real-time multi-color tracking with adjustable distance maintenance.

**Behavior:**

* Tracks **three color IDs (1,2,3)** as one unified color (prevents errors caused by illumination change)
* Yaw alignment uses the **largest bounding box** among the three colors.
* Distance control uses the **sum of areas** from all matching color blocks.
* Uses a **3-frame moving average** to reduce jitter and segmentation noise.

**Useful for:**
Smooth, robust tracking under noisy lighting conditions.

---

### **ram_object.py**

**Goal:** Line up with an object and physically knock it over using a state machine.

**Behavior Overview:**

1. **Search** — Robot rotates until the HuskyLens detects the target object ID.
2. **Align** — Robot rotates left/right until object is horizontally centered.
3. **Ram** — Robot drives forward a fixed distance at high speed.
4. **Finish** — Robot stops after impact.

**Useful for:**
Demonstrations of closed-loop visual alignment and simple state machines.

---

## **Installation & Setup**

1. Clone the repository onto your XRP's Python environment.
2. Ensure XRPLib and qwiic_huskylens are installed.
3. Connect HuskyLens to the Qwiic port (I2C0: SDA=4, SCL=5).
4. Train HuskyLens in Color Recognition or Tag Recognition mode depending on the script.
5. Run any module using:

```python
python3 track_object.py
```

---

## **Future Improvements**

* Dynamic calibration of target distance
* More advanced servo-based pitch tracking
* Potential upgrade to a full CNN running on-device (if hardware allows)