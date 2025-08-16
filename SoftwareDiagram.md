graph TD
    subgraph " "
        subgraph "Operational Modes (Entry Points)"
            direction LR
            A[BlueTeleop.java]
            B[RedTeleop.java]
            C[LeftAuto.java]
            D[RightAuto.java]
            E[LocalizationTest.java]
        end

        subgraph "Central Orchestrator"
            R[Robot.java]
        end

        subgraph "Hardware Subsystems & Abstractions"
            direction TB
            DT{Drivetrain.java}
            MD(MecanumDrive.java)
            ARM(Arm.java)
            CLM(Climber.java)
        end

        subgraph "Localization & Odometry"
            direction TB
            L{Localizer.java}
            TDWL(ThreeDeadWheelLocalizer.java)
        end

        subgraph "Data Transfer Objects (Messages)"
            direction TB
            PM[PoseMessage]
            DCM[DriveCommandMessage]
            MCM[MecanumCommandMessage]
            TCM[TankCommandMessage]
            MLIM[MecanumLocalizerInputsMessage]
            TLIM[TankLocalizerInputsMessage]
            TDWIM[ThreeDeadWheelInputsMessage]
            TWODWIM[TwoDeadWheelInputsMessage]
        end
        
        subgraph "Utilities"
            DRW(Drawing.java)
        end

        %% Define Relationships
        A --> R
        B --> R
        C --> R
        D --> R

        R --> DT
        R --> ARM
        R --> CLM
        
        MD -- Implements --> DT
        MD --> L
        
        L -- Implemented by --> TDWL

        E --> MD
        E --> TDWL

        %% Data Flow (Dashed Lines)
        MD -.-> DCM
        MD -.-> MCM
        TDWL -.-> TDWIM
        TDWL -.-> PM

        %% Utility Usage
        MD --> DRW
        TDWL --> DRW

        %% Styling
        classDef opmode fill:#cde4ff,stroke:#5a7d9e,stroke-width:2px,color:#2c3e50
        class A,B,C,D,E opmode

        classDef robot fill:#ffc8b3,stroke:#c4633a,stroke-width:3px,color:#2c3e50
        class R robot

        classDef subsystem fill:#d5f0e3,stroke:#5a9e7c,stroke-width:2px,color:#2c3e50
        class MD,ARM,CLM subsystem

        classDef abstraction fill:#d5f0e3,stroke:#5a9e7c,stroke-width:2px,color:#2c3e50,stroke-dasharray: 5 5
        class DT,L abstraction

        classDef localizer fill:#fff2cc,stroke:#b38f00,stroke-width:2px,color:#2c3e50
        class TDWL localizer
        
        classDef message fill:#e6ccff,stroke:#8e44ad,stroke-width:1px,color:#2c3e50
        class PM,DCM,MCM,TCM,MLIM,TLIM,TDWIM,TWODWIM message
        
        classDef util fill:#f5f5f5,stroke:#999,stroke-width:2px,color:#2c3e50
        class DRW util
    end
