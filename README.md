# RRT Server Library (librrtserver)

## Installation
librrtserver serves a searching server finds a free path with from 2 inputs, test nodes are compiled using CMake.

### Setup
```bash
cd <librrtserver directory>/cpp
mkdir build && cd build
cmake .. 
make
```

#### Run Executable
To run sample scripts go to `build` folder and launch `./librrtserver_node`, the output in the console print out until `ctrl-C`
```bash
[test_server_module] total runs(194356) and timer(170.542s)
[test_server_module]
START:  3.2284 2.75606 4.57808
END:  -2.8311 -1.74994  1.19071
DISTANCE: 8.27622
[test_server_module] total runs(194357) and timer(170.543s)
[test_server_module]
START: 5.82676 5.09676 2.68211
END: -5.05684  4.33735  1.53228
DISTANCE: 10.9705
[test_server_module] total runs(194358) and timer(170.544s)
[test_server_module]
START:   5.22016 0.0200819   5.49535
END:  3.67211 -4.17925  3.30352
DISTANCE: 4.98347
[test_server_module] total runs(194359) and timer(170.544s)
[test_server_module]
START: 1.11849 5.83069 4.96197
END: 0.701279 -3.82921   1.3927
DISTANCE: 10.3067

```

#### Include in other projects:
To link this lib properly, add following in the `CMakeLists.txt`
```
find_package(librrtserver REQUIRED)
target_link_libraries(${PROJECT_NAME}_XX
  ...
  librrtserver
)
```

### References
1. RRT implementation on 2D https://github.com/swadhagupta/RRT/blob/master/rrt.cpp

