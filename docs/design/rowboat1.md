# Rowboat1 System Design

This document provides an overview of Rowboat1’s mechanical, electrical, and software design.

## Electrical
### Primary computer - Arbiter & Obstacle Detection
The primary computer is an Odroid XU4 [ref]. It runs Lubuntu linux. It houses the arbiter software nodes, as well as all imaging gathering and processing software nodes.

### Secondary computer - Positioning & Navigation
This computer is an Odroid C1+, a less powerful version of the XU4, running Lubuntu linux. It acts as the primary navigation unit— all primary proprioceptive sensors feed into it, and all interface and navigation software nodes run on it.

### Tertiary computer - Safety Monitoring
This computer, like the secondary, is an Odroid C1+, running Lubuntu linux. It has its own power supply because it controls the ballast system, accepts safety sensor input (moisture, temperature, etc), and has the authority to surface the vehicle in cases of emergency.

## Software
### Arbiter

### Positioning

### Navigation

### Environment Mapping

### Obstacle Detection

### Safety Monitor

### Hardware Interfaces
