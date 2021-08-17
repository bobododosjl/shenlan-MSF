## 多传感器融合课程环境安装 NavitePC

### 基础环境
#### 1.Ubuntu 18.04
 * 包括VScode VIM cmake 等常用工具

#### 2.Ros Melodic
 * rosdep init 及 rosdep update 不成功的 可参考网址
 * https://blog.csdn.net/leida_wt/article/details/115120940?utm_source=app&app_version=4.5.8
 * https://github.com/RocShi/rostaller  !!使用一键安装

### 使用库
#### 1. g2o
```shell

// 从github上克隆源码
$ git clone https://github.com/RainerKuemmerle/g2o.git

// 安装依赖
$ sudo apt-get install libeigen3-dev
$ sudo apt-get install libsuitesparse-dev
$ sudo apt-get install qtdeclarative5-dev
$ sudo apt-get install qt5-qmake
$ sudo apt-get install libqglviewer-dev

// 编译
cd g2o
$ mkdir build
$ cd build
$ sudo ldconfig
$ cmake ..
$ make

//安装
$ sudo make install
```
**注：** 一定要在编译前进入build，进行sudo ldconfig</br>
**注：** 若使用最新g2o库，需在在CMakeList中添加

```CMakeList.txt
...

set(CMAKE_CXX_STANDARD 14)

...
```
或安装slam十四讲中的g2o库</br>
git 地址：https://github.com/gaoxiang12/slambook

#### 2. Ceres
```shell
// 需要自己下载源码
下载地址： https://github.com/ceres-solver/ceres-solver/releases  ##version 2.0.0

// 修改 sources.list
$ sudo gedit /etc/apt/sources.list
// 将此地址添加到source.list上
$ deb http://cz.archive.ubuntu.com/ubuntu trusty main universe 

// 更新源
$sudo apt-get update
 sudo apt-get upgrade
// 安装依赖
$ sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3.1.2 libgflags-dev libgoogle-glog-dev libgtest-dev

// 编译及安装,进入下载的ceres包（解压后）
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install

```

#### 3.Geographic
```shell
// 需要自己下载源码
下载地址： https://sourceforge.net/projects/geographiclib/

// 解压并进入文件夹
$ cd GeographicLib-1.51

// 编译及安装
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install

```

#### 4.gflags

```shell
// 克隆源码
$ git clone https://github.com/gflags/gflags
//若克隆失败，可以进github，搜gflags下载
// 解压并进入文件夹
$ cd gflags

// 编译及安装
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install

```

#### 5.glog--_未成功_--已经配置了？

```shell
// 克隆源码
$ git clone https://github.com/google/glog

// 安装依赖
$ sudo apt-get install autoconf automake libtool

// 配置
$ ./autogen.sh
$ ./configure

// 编译及安装
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install

```
#### 6.fmt
```shell
// 克隆源码
$ git clone https://github.com/fmtlib/fmt.git

// 编译及安装
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install

```

#### 7.Sophus(模板类的)

```shell
// 克隆源码
$ git clone https://github.com/strasdat/Sophus.git

// 编译及安装
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install

```
Sophus依赖Eigen，所以二者必须同时存在于CMakeLists
```cmake
cmake_minimum_required( VERSION 2.8)

project( test )

include_directories( "/usr/include/eigen3" )
include_directories( "/usr/include/Sophus" )
add_executable(useSophus useSophus.cpp)

```

#### 8.Google/protobuf

克隆源码
```shell
$ wget https://github.com/google/protobuf/archive/v2.6.1.zip
$ unzip v2.6.1.zip
$ cd protobuf-2.6.1
```

修改autogen
```sh
// 将
    echo "Google Test not present.  Fetching gtest-1.5.0 from the web..."
    curl http://googletest.googlecode.com/files/gtest-1.5.0.tar.bz2 | tar jx
    mv gtest-1.5.0 gtest

// 修改为
    wget https://github.com/google/googletest/archive/release-1.5.0.tar.gz
    tar xzvf release-1.5.0.tar.gz
    mv googletest-release-1.5.0 gtest
```

执行autogen
```shell
$ ./autogen.sh
$ ./configure
$ make
$ make check
$ sudo make install
```

加入路径
```
$ export LD_LIBRARY_PATH=/usr/local/lib
```
