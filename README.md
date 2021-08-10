# Pick-and-Place-with-RL

![example workflow](https://github.com/kuolunwang/Pick-and-Place-with-RL/actions/workflows/main.yml/badge.svg)

## clone repo
```
    git clone --recursive git@github.com:kuolunwang/Pick-and-Place-with-RL.git
```

*You should run this code on a GPU computer*

---

## Building docker image
```
    cd Docker && source build.sh
```

## How to run
```
    cd Docker && source docker_run.sh
    Docker $ cd catkin_ws && catkin_make
    Docker $ source environment.sh
```

## If you want to enter same container
```
    $ cd Docker && source docker_join.sh
    Docker $ source environment.sh
```