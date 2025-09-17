# RobUST Robot Control Framework

A Unity-based real-time control system for cable-driven robotics, specifically designed for the RobUST robot platform. This framework provides Soft real-time deterministic motor control, multi-sensor integration, and tension planning for cable actuation.

## Project Architecture

### Core Design Philosophy

Unlike typical Unity projects where each `MonoBehaviour` operates independently with its own `Update()` loop, this framework implements a centralized control architecture for deterministic real-time robotic control. The system uses a single main control loop with dedicated driver threads for hardware interfaces.

### Key Components

| Script | Function |
|--------|----------|
| `RobotController.cs` | Main control loop coordinating all subsystems |
| `LabviewTcpCommunicator.cs` | TCP interface to LabVIEW motor controllers |
| `TrackerManager.cs` | HTC Vive tracker interface via OpenVR |
| `ForcePlateManager.cs` | Vicon force plate integration via .NET SDK |
| `CableTensionPlanner.cs` | Quadratic programming solver for cable tensions |
| `RobotVisualizer.cs` | Unity scene visualization updates |
| `DataStructures.cs` | Shared data types and utilities |

## Driver Architecture

The framework employs three main driver scripts (`LabviewTcpCommunicator`, `TrackerManager`, `ForcePlateManager`) designed as standalone, reusable components. Each driver:

- Runs in its own thread with high-precision timing using Unity's `System.Diagnostics.Stopwatch`
- Maintains pseudo-deterministic sampling frequencies
- Uses data locks for thread-safe communication with the main control loop
- Can be integrated into other robotic projects without modification

### `LabviewTcpCommunicator.cs`

Manages communication with the low-level motor controller running on a PXIe system:

- **Frequency**: 1 kHz TCP transmission
- **Configuration**: Motor indices set during initialization
- **Interface**: `UpdateTensionSetpoint(double[] setpoints)` for real-time updates
- **Protocol**: Sends motor index and tension setpoint pairs via TCP
- **Port**: 8052 (configurable)

### `TrackerManager.cs`

Interfaces with HTC Vive tracking system for pose estimation:

- **Frequency**: 90 Hz (limited by HTC hardware)
- **API**: OpenVR for direct access to tracker transformation matrices
- **Configuration**: Supports 3 trackers (frame reference, end-effector, center of mass)
- **Setup**: Automatically discovers connected trackers and displays serial numbers for configuration
- **Coordinate System**: Uses HTC Vive tracker orientation guidelines

### `ForcePlateManager.cs`

Integrates with Vicon force measurement systems:

- **Status**: Work in progress
- **Interface**: Vicon DataStream SDK via .NET DLL packages
- **Architecture**: Follows same threaded driver pattern as other components

## Physics and Control

### CableTensionPlanner

Solves the cable tension optimization problem using quadratic programming:

- **Solver**: Alglib Convex QP implementation (dense IPM)
- **Objective**: achieve desired wrench while minimizing parasitic wrench and maintaining minimum cable tensions
- **Configuration Parameters**:
  - Number of cables
  - Chest anteroposterior distance
  - Chest mediolateral distance
  - Belt size (small/medium/large)
- **Primary Function**: `CalculateTensions(Matrix4x4 endEffectorPose, Vector3 desiredForce, Vector3 desiredTorque, Matrix4x4 robotFramePose)`

### RobotVisualizer

Handles Unity scene updates and coordinate frame transformations for visualization only:

- **Coordinate Conversion**: Right-handed tracker data to left-handed Unity coordinate system
- **Initialization**: Captures single frame tracker snapshot for reference positioning
- **Camera Setup**: Positions Unity camera relative to frame tracker for optimal viewing

## Setup and Installation

### Prerequisites

- Unity Hub with Unity Editor installed
- Steam account with SteamVR
- HTC Vive base stations and headset
- 3 HTC Vive trackers

### Quick Start

1. Clone Repository
2. Open Project in Unity
- Launch Unity Hub
- Select "Open Project"
- Navigate to and select the "Darren RobUST Controller" folder
3. Load Scene
- Open the "Robot Controller Scene" in Unity
4. Configure SteamVR
- Install SteamVR through Steam
- Connect and power on base stations and headset
- Pair the 3 Vive trackers using "Pair Controller" in SteamVR
- Verify tracker status icons are lit in SteamVR interface
5. Configure Tracker Serial Numbers
- Run the project once to see discovered tracker serial numbers in console
- Copy serial numbers to appropriate variables in TrackerManager:
  - Frame tracker serial
  - End-effector tracker serial
  - Center of mass tracker serial

Testing Without Hardware
For development and testing without the full LabVIEW motor control system:
**Simulate Local TCP Listener**
```bash
ncat -l 8052
```
This will display incoming motor commands for verification of communication protocols.

### Dependencies
- Unity: 2021.3 LTS or newer recommended
- SteamVR: Latest version through Steam
- OpenVR: Included with SteamVR installation
- Vicon DataStream SDK: .NET packages (for force plate integration) ---> Plugins Folder
- Alglib: Included for quadratic programming solver ---> Plugins Folder

### Usage Notes
- Ensure all trackers are awake before starting
- Frame tracker position is captured once during initialization and used as reference
- System requires SteamVR to be running and trackers connected before Unity execution
- Motor indices must be configured before runtime - no dynamic motor discovery supported


