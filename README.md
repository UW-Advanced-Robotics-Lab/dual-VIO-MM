# Dual-VIO-MM 
This work is a tailored dual-VIO for a mobile manipulator with arm kinematics.
The Dual-VIO-MM is an optimization based tightly coupled multi-VIO fusion with arm kinematics.
This work contains a baseline dual-VIO (a multi-threaded VINS-Mono), and a proposed dual-VIO-Arm (a multi-threaded VINS-Mono with Arm Odometry).

## (Internal) Evaluation Script:
- For hardware setup, refer to https://github.com/UW-Advanced-Robotics-Lab/lab-wiki/wiki/Waterloo-Steel%3APlatform-Instruction
- Please refer to https://github.com/jaku-jaku/vins-research-pkg
- batch scripts to run over all recorded ros bags:
  ```
  $HOME/UWARL_catkin_ws/src/waterloo_steel/waterloo_steel_demo/waterloo_steel_analyzer/shortcuts/batch_tmux_vins.sh waterloo_steel_demo_0519 mono_rgb_imu EE d455 all all accurate_T_ic -1 -1 && /home/jx/UWARL_catkin_ws/src/waterloo_steel/waterloo_steel_demo/waterloo_steel_analyzer/shortcuts/batch_tmux_vins.sh waterloo_steel_demo_0519 mono_rgb_imu base d455 all all accurate_T_ic -1 -1 
  ```

## Citation - BibTeX:
- Public available on ArXiv (Submission to IEEE TMECH): https://arxiv.org/abs/2407.13878
- IEEE TMECH / AIM 2024 Focused Section (Final): https://ieeexplore.ieee.org/document/10542393
```
@article{Xu_2024,
   title={A New Tightly-Coupled Dual-VIO for a Mobile Manipulator With Dynamic Locomotion},
   ISSN={1941-014X},
   url={http://dx.doi.org/10.1109/TMECH.2024.3400918},
   DOI={10.1109/tmech.2024.3400918},
   journal={IEEE/ASME Transactions on Mechatronics},
   publisher={Institute of Electrical and Electronics Engineers (IEEE)},
   author={Xu, Jianxiang and Jeon, Soo},
   year={2024},
   pages={1–9} }
```

## Acknowledgements
- This Repo is based on https://github.com/HKUST-Aerial-Robotics/VINS-Fusion
- Uses [ceres solver](http://ceres-solver.org/) for non-linear optimization and [DBoW2](https://github.com/dorian3d/DBoW2) for loop detection, a generic [camera model](https://github.com/hengli/camodocal) and [GeographicLib](https://geographiclib.sourceforge.io/).

## License
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.
