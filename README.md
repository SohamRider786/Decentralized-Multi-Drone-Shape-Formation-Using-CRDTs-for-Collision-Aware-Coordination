# Decentralized-Multi-Drone-Shape-Formation-Using-CRDTs-for-Collision-Aware-Coordination


This project simulates a **decentralized swarm of drones** transitioning from an **inverted V formation** to a **circular formation**, with real-time collision handling and CRDT-based state synchronization.

---

## ✨ Features

- 🚁 **Multithreaded Simulation:** Each drone runs as an independent thread, computing its path and state.
- 🔁 **CRDT-based Synchronization:** Drones share and merge state using Conflict-Free Replicated Data Types (CRDTs).
- 🛑 **Collision Handling:**
  - `BOUNCE` — drones attempt to recover by bouncing to random nearby positions.
  - `DEAD` — drones stop and mark themselves dead on collision.
- 🧠 **Dynamic Formation Switching:**
  - Starts in an **inverted V shape**
  - Transitions into a **circular formation**
- 📊 **Performance Logging:** Outputs detailed CSV data of positions, collision count, and final statuses.
- 📍 **Python Visualization:** Static visual plots of initial and final formations.

---

## 🧱 Technologies Used

- **C++17**
  - `std::thread`, `mutex`, `atomic`, `condition_variable`
- **Python 3**
  - `matplotlib`, `pandas` (for post-simulation plotting)
- **CSV Output**
  - For trajectory and final position logging

---

## 📂 Project Structure

.  
├── main.cpp # C++ simulation entry point  
├── drone.hpp # Drone class logic and threading  
├── drone_positions.csv # Output: final positions of all drones  
├── results.csv # Output: average collisions and deaths  
├── visualize.py # Python script for plotting V and Circle  
