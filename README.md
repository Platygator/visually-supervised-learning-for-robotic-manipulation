# Robotic Manipulation using Linear Genetic Programming and Visual Feedback

This repository is for the project "Robotic Manipulation using Linear Genetic Programming and Visual Feedback" in the course "Humanoid Robotics (TIF160)" at the Chalmers University of Technology.

## Dependencies
* [Libcluon](https://github.com/chrberger/libcluon)

## Installing
Make sure libcluon is installed on your system

`sudo add-apt-repository ppa:chrberger/libcluon`

`sudo apt upgrade`

`sudo apt install libcluon`



create a virtual environment with all dependencies using

`make`

## Run

Evaluation mode, which uses the best LGP chromosome to try to grab an object

`make run_evaluate`

Training mode with...

a) software-only-mode

`make run_software`

b) hardware in the loop

`make run_hardware`


## Cleanup

`make clean`

## Authors 

* Aren Moosakhanian 
* Jan Paul Theune 
* Jan Schiffeler
* Sourab Bapu Sridhar

## License

This project is released under the terms of the [MIT License](LICENSE)
