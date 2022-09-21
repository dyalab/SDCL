Required Packages:
1. OMPL, https://ompl.kavrakilab.org/index.html, for planner setup
4. OpenCV, https://opencv.org/, for 2d environment setup. 
5. ThunderSVM, https://github.com/Xtra-Computing/thundersvmfor, for learning Kernel-SVM.
6. Amino, https://github.com/golems/amino, for scene setup, viusalization and robot modeling.
7. NLOPT, https://nlopt.readthedocs.io/en/latest/, for solving optimization problems. 

Build:
 ```mkdir build```
 ```cd build```
 ```cmake ..```
 ```make```

Run:
```./RunSDCL <scene_number> <timeout-limit> <use_training> <use_Gaussian>```
e.g. ```./RunSDCL 2 100 True True```
