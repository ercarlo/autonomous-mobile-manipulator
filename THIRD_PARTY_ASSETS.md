# Third-Party Assets

This project incorporates third-party 3D assets for robotic simulation in Gazebo Harmonic.

The assets retain their respective original licenses. The repository's software license does not override the licensing terms of third-party models.

## 1. YCB Object and Model Set

**Original dataset:** Yale-CMU-Berkeley (YCB) Object and Model Set  
**Additional asset source:** AI Habitat YCB Dataset  
**License:** Creative Commons Attribution 4.0 International (CC BY 4.0)

### Included Objects

| Model | YCB Identifier | Directory |
|---|---|---|
| Apple | 013_apple | `ycb_apple/` |
| Mug | 025_mug | `ycb_mug/` |
| Bowl | 024_bowl | `ycb_bowl/` |
| Mustard Bottle | 006_mustard_bottle | `ycb_mustard_bottle/` |

### Modifications and Integration

The assets have been integrated into the Autonomous Mobile Manipulator simulation environment.

Project-specific adaptations include:

- Gazebo Harmonic SDF model descriptions.
- Simplified collision geometries.
- Approximate physical and inertial properties where required.
- Model positioning and integration into the robotic manipulation scene.

The original YCB dataset and AI Habitat distribution provide the licensing and provenance references for the visual assets.

**References:**

- YCB Object and Model Set: https://www.ycbbenchmarks.com/
- AI Habitat YCB Dataset: https://huggingface.co/datasets/ai-habitat/ycb
- CC BY 4.0 License: https://creativecommons.org/licenses/by/4.0/

## 2. Industrial Workbench

**Model:** TRADE 160 – Commercial Workbench  
**Original creator:** Innerscene  
**License:** Creative Commons CC0 1.0

**Included asset:**

`industrial_workbench/meshes/commercial-workbench.glb`

The visual model is integrated into Gazebo Harmonic using a custom SDF description.

Project-specific modifications include collision geometry, physical properties and positioning within the simulated industrial environment.

**Source:**

https://www.innerscene.com/tools/library/3d-parts/trade-160-commercial-workbench-eb987083

**License:**

https://creativecommons.org/publicdomain/zero/1.0/

## 3. Robot Descriptions

The robotic platform integrates external descriptions of:

- MiR100 mobile robot.
- Universal Robots UR5e manipulator.
- Robotiq 2F-85 gripper.

These resources are maintained separately from the project's custom ROS 2 packages.

Their original licenses and redistribution requirements must be respected.

## 4. Attribution and Licensing

All third-party assets remain the property of their respective copyright holders where applicable.

The project provides attribution to the original datasets and creators and identifies modifications introduced for simulation.

Redistribution of individual meshes is conditional on confirming their provenance and applicable licenses.

No endorsement by the original creators or dataset maintainers is implied.
