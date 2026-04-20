# Proposed Metrics for Multi-Robot Boundary Filling Analysis

To effectively compare the performance of various Artificial Potential Field (APF) and Harmonic Potential Field (HPF) implementations in your research paper, you should use a multi-dimensional metric suite. This allows you to quantify not just "how fast" a swarm performs, but also the "quality" of its coordination and the "energy efficiency" of the agents.

Below are the recommended metrics categorized by their focus area.

---

## 1. Temporal & Efficiency Metrics
These measure the raw performance and speed of the swarm in completing the mission.

*   **Total Completion Time (TCT):** The total number of timesteps until the final robot enters a "Parked" state. 
    *   *Significance:* Directly measures the overall efficiency of the algorithm.
*   **Average Convergence Time (ACT):** The mean time taken per robot to reach its assigned target.
    *   *Significance:* Helps identify if a few "late" robots are dragging down the swarm performance or if the whole swarm moves slowly.
*   **Time-to-First-Entry (TFE):** Timestep when the first robot successfully parks.
    *   *Significance:* Measures the "latency" or startup efficiency of the system.

## 2. Kinetic & Energy Metrics
These measure how much "work" the robots are doing and how smooth their paths are.

*   **Total Path Length (TPL):** The sum of the Euclidean distances traveled by all robots over the entire simulation.
    *   *Significance:* Represents the energy consumption of the swarm; shorter paths are generally more "optimal."
*   **Average Velocity ($\bar{v}$):** The mean speed of the robots across the whole run.
    *   *Significance:* Higher average velocity usually indicates fewer "stuck" states or confusion periods (especially useful for comparing the **Adaptive Bias** method).
*   **Path Tortuosity (Straightness Index):** The ratio of the shortest possible distance (Spawn to Target) to the actual Path Length. 
    *   *Significance:* Measures path efficiency. A value of 1.0 is a perfect straight line; lower values indicate jitter, collision avoidance maneuvers, or wandering.

## 3. Coordination & Conflict Metrics
These measure how well the robots interact and manage congestion.

*   **Inter-Robot Distance Variance:** The variance of distances between all pairs of neighboring robots during the approach phase.
    *   *Significance:* High variance suggests "clumping" or congestion; lower variance suggests a more ordered, well-spaced "columnar" flow (often seen in the **V-shaped Gradient** approach).
*   **Mean Separation Distance:** The average distance maintained between robots during travel.
    *   *Significance:* Indicates the safety margin and the effectiveness of the robot-robot repulsion field.
*   **Stall Frequency (Confusion Count):** Number of timesteps where a robot's net force magnitude falls below a certain threshold (e.g., your `CONFUSION_THRESHOLD`).
    *   *Significance:* This is a **critical metric** for your paper to justify why the **Adaptive Bias** or **Harmonic Field** methods were introduced.

## 4. Packing & Order Metrics
These measure the quality of the "filling" process.

*   **Sequential Packing Order Accuracy:** A measure of whether the robots filled the targets in the intended priority (e.g., Outside-In vs. Random).
    *   *Significance:* Compares your **V-shaped Gradient** vs. **Left-to-Right Gradient** vs. **Equal Forces**.
*   **Target Switching Frequency:** How many times a robot changes its "Guidance Point" before finally parking.
    *   *Significance:* Measures the stability of the task assignment logic. High switching usually leads to jittery, inefficient swarm behavior.

---

## Summary Comparison Matrix (Theoretical Expectation)

| Metric | Equal Forces | V-Gradient (Original) | Adaptive Bias | Harmonic Field |
| :--- | :---: | :---: | :---: | :---: |
| **Completion Time** | High (Jams) | Medium-Low | Low | Very Low |
| **Path Tortuosity** | High | Medium | Low | Very Low |
| **Stall Frequency** | Very High | High | Very Low | Zero |
| **Energy (TPL)** | High | Medium | Low | Lowest |

> [!TIP]
> **Recommendation for your paper:** Focus the narrative on how the **Stall Frequency** decreases as you move from "Equal Forces" $\rightarrow$ "Gradients" $\rightarrow$ "Adaptive Bios" $\rightarrow$ "Harmonic Fields". This provides a clear "evolution of logic" for your research story.
