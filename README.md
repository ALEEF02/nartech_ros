# nartech_ros

<img width="1071" alt="Image" src="https://github.com/user-attachments/assets/6fcfeaab-55b6-4637-900d-85dc41922a97" />

### Start Unitree G1 MuJoCo simulation (optional helper script)

```bash
./start/start_unitree_g1_sim.bash
```

This runs:

```bash
python3 deploy/deploy_mujoco/deploy_mujoco.py g1.yaml
```

Set `UNITREE_ROOT` if your `unitree_rl_gym` repo is not adjacent to `nartech_ros`.

### Start NARTECH + Nav2 + SLAM + scan fusion stack (G1-first)

```bash
./start/start_nartech.bash
```

This launch starts:
1. `pointcloud_to_laserscan` (Livox -> `/scan/livox`)
2. `depthimage_to_laserscan` (D435i depth -> `/scan/depth`)
3. `scan_mux_selector` (`/scan/livox` primary, `/scan/depth` fallback, output `/scan`)
4. `nav2_bringup` with SLAM enabled
5. `nartech_main` (arm controller disabled by default for G1 phase)

### Run with MeTTa demo script

```bash
./start/start_nartech.bash metta demo_with_nars.metta
```

This opens the demo file and runs:

```bash
python3 main.py ./demos/demo_with_nars.metta
```

### Test various MeTTabridge plugins without running ROS:

```bash
python3 mettabridge.py ./plugins/Xplugintests/file.metta
```

These plugins can also be used standalone as MeTTa extensions via the MeTTa import command.

### ROS2 contract

Canonical G1 topic/frame contract is in:

`config/g1_ros_contract.yaml`
