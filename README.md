# Stereo Calibration


## Extract intrinsic and extrinsic properties of a monocular

This camera calibration was done by using *OpenCV*. This is just a simple test done by using random chessboard images found on Internet.

## Validating Estimates vs Ground Truth

Run the simulation:

`./Examples/Stereo/stereo_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo/EuRoC.yaml ~/Datasets/EuRoC/MH01 ./Examples/Stereo/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereo`


Then, compare with Ground Truth

`python2.7 evaluation/evaluate_ate_scale.py --verbose evaluation/Ground_truth/EuRoC_left_cam/MH01_GT.txt f_dataset-MH01_stereo.txt --plot MH01_stereo.pdf`

With the *--verbose argument*, it gives this kind of output:

<pre>

compared_pose_pairs 3638 pairs
absolute_translational_error.rmse 0.035956 m
absolute_translational_error.mean 0.033244 m
absolute_translational_error.median 0.036838 m
absolute_translational_error.std 0.013701 m
absolute_translational_error.min 0.002904 m
absolute_translational_error.max 0.104954 m
max idx: 1380

</pre>

The plot of the trajectory is saved in *MH01_stereo.pdf*


## Bibliography

- OpenCV documentation, https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
