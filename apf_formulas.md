# Artificial Potential Field (APF) Formulas

This document outlines the mathematical formulas used to enable collision-free swarm robot movement. These formulas are based on the foundational framework introduced in the [Khatib 1986 IJRR](Khatib_1986_IJRR.pdf) paper.

## 1. Robot Motion Model
Each robot's movement is determined by the total force exerted on it in each timestep.

$$F_{total} = F_{random} + F_{robot\_repulsion} + F_{wall\_repulsion}$$

The velocity is derived by normalizing $F_{total}$ and scaling it by the clamped `speed`, ensuring robots do not exceed their maximum physical velocity limit:
$$v = \frac{F_{total}}{||F_{total}||} \cdot \min(||F_{total}||, speed)$$

## 2. Component Forces

### Exploration Force ($F_{random}$)
Maintains the original random walk behavior, providing a constant forward propulsion force in the robot's currently selected random direction ($\theta$):
$$F_{random} = [ speed \cdot \cos(\theta), speed \cdot \sin(\theta) ]$$

### Robot Repulsion Force ($F_{robot\_repulsion}$)
Prevents robots from colliding. It acts between any two robots whose shells are closer than the sensing radius $R_{robot}$. Let $d$ be the effective shortest distance between the perimeters of two robots (center distance minus $2 \cdot radius$).

If $d < R_{robot}$:
$$F_{repulsion} = k_{rep} \cdot \left(\frac{1}{d} - \frac{1}{R_{robot}}\right) \cdot \frac{1}{d^2} \cdot \vec{u}_{away}$$

Where:
* $k_{rep}$ is the repulsion gain constant for robots.
* $\vec{u}_{away}$ is the normalized unit vector pointing from the neighbor to the current robot.

### Wall Repulsion Force ($F_{wall\_repulsion}$)
Prevents robots from crashing into the boundaries or the inner U-shaped obstacle. It acts between a robot and a line segment when its distance $d$ from the wall to the robot shell is less than $R_{wall}$.

If $d < R_{wall}$:
$$F_{wall} = k_{wall} \cdot \left(\frac{1}{d} - \frac{1}{R_{wall}}\right) \cdot \frac{1}{d^2} \cdot \vec{u}_{away\_from\_wall}$$

Where:
* $k_{wall}$ is the repulsion gain constant for walls.
* $\vec{u}_{away\_from\_wall}$ is the normalized unit vector pointing perpendicularly away from the wall segment towards the robot.
