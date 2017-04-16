## Extended Kalman Filter

### Fuse LIDAR and radar to track object

---


[//]: # (Image References)
[out1]: ./images/out1.png
[out2]: ./images/out1.png


### How to use it

download this project:

`git clone 'thisrepo'`

change directory:

`cd thisrepo/build`

compile the project (optional)

`cmake .. && make`

run the executable with input and output files specified:

`./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt ../data/out.txt`

### Accuracy result
#### For input data 'sample-laser-radar-measurement-data-1.txt':

Accuracy - RMSE:
[0.065165, 0.0605294, 0.5497, 0.544984]

![alt text][out1]


#### For input data 'sample-laser-radar-measurement-data-2':

Accuracy - RMSE:
[0.185495, 0.190302, 0.487137, 0.810657]

![alt text][out2]

###  Algorithm details

* This project is built in Oo style following the sample pipeline provided by Udacity
* The first measurements are used to initialize the state vectors, and covariance matrices are initialized to
`1, 0, 0, 0,
0, 1, 0, 0,
0, 0, 1000, 0,
0, 0, 0, 1000;`
* Upon receiving a measurement after the first, the algorithm predicts object position to the current timestep and then update the prediction using the new measurement. If the measurements of both x and y position are zero, the update step will be skipped.
* The algorithm uses different update functions based on the type of measurements (LIDAR or radar).
