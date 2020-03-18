# About
demo - MATLAB code for our CVPR 2020 paper:

"Globally Optimal Contrast Maximisation for Event-based Motion Estimation", CVPR 2020 (**Oral**), [pdf](https://arxiv.org/abs/2002.10686)

[Daqi Liu](https://sites.google.com/view/daqiliu/home), [Álvaro Parra](https://cs.adelaide.edu.au/~aparra/) and [Tat-jun Chin](https://cs.adelaide.edu.au/~tjchin/doku.php?id=start).

# Description
demo.m runs our CMBnB and CMGD (contrast maximization with conjugate gradient, our impelmentation) on a subsequence from dynamic with about 19,000 
events. To execute the demo just run demo.m in MATLAB. The script will:
* show the input stream and the event image without motion compensation,
* run CMBnB and CMGD, and
* plot motion compensated images for CMBnB and CMGD.

The expected runtime of the demo is less than 1 minute on a standard 
desktop PC.

Demo and code have been tested under
* Ubuntu 18.04
* MATLAB R2019a
* GCC v7

# Dependencies
[eigen3 library](http://eigen.tuxfamily.org/index.php?title=Main_Page)

This library is included.

# Dataset 
Full dynamic, poster and boxes dataset can be downloaded [here](http://rpg.ifi.uzh.ch/davis_data.html)

Full Star tracking dataset can be downloaded [here](https://cs.adelaide.edu.au/~tjchin/startracking/)

# Compilation and Run

Binaries for Linux (Ubuntu) are supplied. To compile new binaries, please
run compile.m in MATLAB. 
Code runs on full sequence of event is coming soon

# Support
If you have any questions/bugs to report, please feel free to contact me
