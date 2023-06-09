# [Librebot](https://fall22cs491.github.io/)
Library Robot Model designed as a part of the CS 491/2 Senior Project


Organizing books in a library and keeping them organized is a tedious and time-consuming job that humans still perform. The time and effort put into finding a desired book in the library by a library user increases as the number of books and categories increase in the library. In the age of the fourth industrial revolution, where autonomous robots became a significant part of the workforce in many sectors, the use of robots in this daunting job is still very limited. Our project’s purpose is to model a robot that can be used to automate the organization of library books taking advantage of state-of-the-art technologies. The robot will be able to perceive its environment and detect books using computer vision, identify books by radio frequency identification (RFID) tags or call numbers, and collect from or place them on appropriate shelves, according to the library classification system, using the state-of-the-art machine learning technologies for task and motion planning (TAMP).

# Running the Project Locally
To install and use the project:

Clone the project 
```bash
$ git clone https://github.com/Fall22CS491/LibreBot
```
To run the notebooks you need to use the docker image provided at
https://hub.docker.com/r/hikmetsimsir/visionstorage2iiwa.

You should pull the image using the following command
```bash
$ docker pull hikmetsimsir/visionstorage2iiwa:latest
```

Then you need to run the docker image and mount the current directory to the docker image.

You can use following scripts to do it
```bash
$ docker run -it -p 8080:8080 -p 7000-7010:7000-7010 --rm -v "$(pwd)/."":"/code visionstorage2iiwa:latest
```

If you won't use computer vision you can use the this (previous version)
```bash
$ sudo ./docker_run_notebook.sh .
```
This will start the docker image and mount the current directory to the docker image. 
It will also start the jupyter notebook server and print the url (with token) to connect to the server.

