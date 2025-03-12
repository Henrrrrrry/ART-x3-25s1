# Client Meeting Minutes






#### Date: 12/03/2025
#### Client: Sui Jackson
#### Attendees: Qianwen Shen, Jia hou, Muchen Li, Hongyu Li, Leliang Wang, Zijun Zhou
#### Minute Taker: Qianwen Shen
***
## Agenda
1. Progress update
2. Presentation of movement solution options
3. Client feedback and decision
4. Hardware discussion
5. Next steps

## Discussion Points

### 1. Progress Update
- Team reported completion of user stories and PBIs
- Presented documentation of work completed since last meeting

### 2. Movement Solution Options
Two possible tracking solutions were presented to the client:
- **Option 1:** Centered Tracking
  - Detected person remains centered in frame at all times
  - Camera moves immediately when person moves
  - Continuous tracking and adjustment

- **Option 2:** Threshold-Based Tracking
  - Define a scaled-down area within the screen as a "safe zone"
  - Camera only moves when person's center point exceeds this area
  - Reduces constant minor adjustments

### 3. Client Feedback
- Client preferred Option 1 (Centered Tracking)
- Rationale: Client wants the camera and spotlight to follow the person continuously
- Spotlight will be positioned adjacent to the camera and move together with it

### 4. Hardware Discussion
- Discussed OpenMV server shield implementation
- Confirmed that spotlight will be mounted next to the camera
- Both components will move in unison to track subjects

### 5. Next Steps
- Begin implementation of the centered tracking solution
- Continue with planned sprint activities
- Prepare hardware integration plan

## Action Items
- Talking about the algorithms

## Next Meeting
19/03/2025