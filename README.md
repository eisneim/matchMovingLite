# matchMovingLite
working in progress

compile with cmake
```
$ mkdir build && cd build
$ cmake ..
$ make
```

compile with g++
```
g++ -std=c++11 src/Tracker.cpp main.cpp -o mm `pkg-config --cflags --libs opencv`
```

