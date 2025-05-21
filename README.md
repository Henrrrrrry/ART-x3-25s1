# ART-x3-25s1

# SpotlightTrack

## Automated Motion Tracking System

SpotlightTrack is an innovative automated motion tracking system designed for interactive installation artists, capable of transforming ordinary walkways into engaging interactive spaces. By intelligently tracking and highlighting people within an area, SpotlightTrack creates a unique atmosphere of perceived presence and social connection.

### Product Overview

SpotlightTrack combines advanced motion tracking technology with interactive capabilities, providing an easy-to-use solution that can be deployed without specialized technical knowledge. The system can:

- Seamlessly follow people's smooth, continuous movements
- Intelligently switch between multiple subjects
- Create dynamic interactive experiences

### Key Features

- **Simple Deployment**: Plug-and-play design minimizes installation time
- **Autonomous Operation**: Once set up, the system runs independently without continuous monitoring
- **High Adaptability**: Suitable for various space sizes and layouts
- **High Reliability**: Stable tracking algorithms ensure consistent performance

### Technical Specifications

- Based on OpenMV H7 Plus camera module for image processing
- Uses PCA9685 PWM driver chip for servo control
- Two FS90R servo motors for horizontal and vertical tracking
- Tracking based on Haar feature classifier for upper body detection
- Resolution: QVGA (320x240)
- Response time: <100ms
- Operating temperature: 0°C to 40°C
- [**Technical Document**](technicalDocument.md)

### Applications

- Art installation exhibitions
- Interactive performance spaces
- Museum exhibits
- Immersive experience design
- Event space enhancement

### Installation Guide

1. **Hardware Setup**
   - Mount OpenMV camera module above target area
   - Connect PCA9685 driver to OpenMV via I2C
   - Attach FS90R servos to the mounting system
   - Adjust camera angles to cover desired space

2. **Software Configuration**
   - Use OpenMV IDE to upload tracking code to the camera module
   - Ensure "haarcascade_upperbody.cascade" file is saved on the OpenMV's SD card
   - Calibrate the system to match space dimensions
   - Set tracking parameters for optimal performance

3. **Testing and Adjustment**
   - Perform initial movement tests to verify tracking accuracy
   - Check servo response to different tracking scenarios
   - Adjust tracking sensitivity if needed

### System Operation

- The system uses grayscale image processing to detect upper body shapes
- IoU (Intersection over Union) algorithm maintains consistent subject tracking
- Servo motors adjust based on calculated position error:
  - Small error (< 15 pixels): Slow or no movement
  - Large error (> 40 pixels): Faster adjustment
- Motors automatically stop when no subject is detected

### Maintenance and Support

- Quarterly system checks recommended
- Firmware updates delivered via USB using OpenMV IDE
- Technical support available through dedicated portal

### About Us

SpotlightTrack was developed by a group of artists and engineers passionate about interactive design and immersive experiences. We are dedicated to creating tools that enable artists to easily transform their visions into reality while creating memorable audience experiences.

### Contact Information

- Email:
- Muchen Li: u7709835@anu.edu.au
- Hongyu Li: u7776180@anu.edu.au
- Jia Hou: u7731692@anu.edu.au
- Zijun Zhou: u7619416@anu.edu.au
- Qianwen Shen: u7726387@anu.edu.au
- Leliang Wang: u7692205@anu.edu.au

---
© 2025 SpotlightTrack. All rights reserved.
