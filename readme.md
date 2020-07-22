1. test psc node :
roslaunch psc test_psc.launch
2. There are some user defined values such as dt, weights for cost functions. You can change them in PSC.cpp
3. As our function to calculate clearance is related with DWA, you need to use your own function to calculate clearance. Currently this parameter is set to 1 (Please see details in PSC.cpp)
