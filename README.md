# Blue Eagle: Autonomous Point Tracking & Navigation

**Blue Eagle** is an advanced autonomous drone project developed in the **Webots** simulator. This phase of the project implements a full **Navigation Pipeline**, allowing the quadcopter to autonomously visit a series of predefined GPS waypoints and return to its starting position (Home) upon mission completion.

## 🚀 Mission Capabilities
*   **Precision Take-off:** Stable vertical ascent to a calibrated target altitude.
*   **GPS Waypoint Tracking:** Autonomous traversal through a series of 4 coordinate points in the 3D map.
*   **State-Machine Logic:** Sophisticated mission management using a Finite State Machine (FSM).
*   **Automated Braking & Hovering:** Intelligent deceleration when approaching target points to ensure positional accuracy.
*   **RTH (Return to Home):** Automatic calculation of the home vector and safe landing sequence after the last waypoint.

---

### 📸 Visual Documentation


#### 1. Flight Navigation State
The drone in mid-flight, executing a rotation maneuver toward the next coordinate.
![Autonomous Flight](./protos/Screenshot%2026-05-02%004020.png)

---

## 🏗 System Intelligence (FSM)

The controller operates on a **Finite State Machine** to manage mission phases:

| State | Phase | Description |
| :--- | :--- | :--- |
| **0** | **Take-off** | Vertical climb and "Home" coordinate registration. |
| **1** | **Heading Alignment** | Calculating the angle to the target using `atan2` and rotating the drone. |
| **2** | **En Route** | Forward flight using pitch manipulation proportional to the distance. |
| **3** | **Braking/Stabilizing** | Applying counter-pitch to halt momentum at the target point. |
| **4** | **RTH & Landing** | Returning to initial GPS coordinates and initiating a slow descent. |

---

## 🛠 Advanced Control Logic

### 1. Vector Navigation
The drone calculates its path using the Euclidean distance and the heading angle:
* **Distance Calculation:** `math.sqrt((Δx)² + (Δy)²)`
* **Heading (Yaw) Control:** The drone aligns its nose toward the target before moving, ensuring efficient forward flight.

### 2. Velocity-Aware Pitch Control
To prevent overshoot, the `desired_pitch` is modulated by a `speed_factor`:
* `speed_factor = min(dist / 3.0, 1.0)`
* This creates a "smooth arrival" effect where the drone slows down as it gets closer to the target.

### 3. Integrated PID Mixer
The mixing matrix now includes a **Yaw Input** to manage rotation:
* **m1 (FL):** `vert + roll - pitch - yaw`
* **m2 (FR):** `vert - roll - pitch + yaw`
* **m3 (RL):** `vert + roll + pitch + yaw`
* **m4 (RR):** `vert - roll + pitch - yaw`

---

## 👨‍💻 Author
**Mohamed Arif Mahyoub Haider**
*Electrical Engineer - Computer and Industrial Control*

---


