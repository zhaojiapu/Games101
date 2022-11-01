# Games101 Assignments

Games101 现代计算机图形学 作业代码
开发环境：
> MacOS Ventura 13.0（Apple Silicon）
> CLion 2022.2
> clang 14.0

首先安装必要的包
> CMake：
> `brew install cmake`
> Eigen：
> `brew install eigen`
> OpenCV：
> `brew install opencv`

然后打开项目，修改CMakeLists.txt
`include_directories("/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3/")`
> 这里注意 M1 芯片的 Mac 与 Intel 芯片不同，homebrew 安装的包都在 /opt/homebrew/ 这个目录下

同时也可以修改项目名称之类的，自行选择～

随后对项目代码中的所有
`#include<eigen3/Eigen/Eigen>`
修改为`#include<Eigen/Eigen>`

然后就可以顺利通过编译啦～


