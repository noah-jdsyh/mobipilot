
# Third-party library dependency
## linux
```shell
sudo apt install make
sudo apt install python
sudo apt install clang
sudo apt install gcc
sudo apt install g++
sudo apt install libssl-dev
sudo apt install python-dev

# To make the apps/tools/parse/ source, you may need to install the following packages:
sudo apt install python3-dev
sudo apt install libeigen3-dev
sudo apt install libpcap-dev

# The eigen library is installed by default in the/usr/include/eigen3/Eigen path
# and needs to be mapped to the/usr/include path using the following command
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen

# To make the src/ws_utils/ source, you may need to install the following packages:
wget https://boostorg.jfrog.io/artifactory/main/release/1.78.0/source/boost_1_78_0.tar.gz
tar -zxvf boost_1_78_0.tar.gz
cd boost_1_78_0
./bootstrap.sh --prefix=/usr/
./b2 install

# Add environment variables
export BOOST_INC=-I/opt/boost1.78/include/
export BOOST_LIB=-L/opt/boost1.78/lib/
export CPLUS_INCLUDE_PATH=/opt/boost1.78/include/:$CPLUS_INCLUDE_PATH
export NO_ROS=1

# HOW to COMPILE
cd xxxx/project/build
./compile_unix.sh
```

## mingw
```shell
# Create a directory called win_build_package in D:\
mkdir win_build_package

# Install MSYS2 into D:\win_build_package
https://www.msys2.org/

# Install the necessary toolchain and library files containing dynamic link libraries under Windows by a terminal which will launch after installing
pacman -S --needed base-devel mingw-w64-x86_64-toolchain mingw64/mingw-w64-x86_64-dlfcn

# Open "MSYS2 MinGW x64" to install some Basic development tools
pacman -S make cmake vim git
pacman -S --needed mingw64/mingw-w64-ucrt-x86_64-boost

# Install pexports.exe into D:\win_build_package\msys64\mingw64\bin
https://sourceforge.net/projects/mingw/files/MinGW/Extension/pexports/

# Install 7-zip into D:\win_build_package
https://www.7-zip.org/

# Install visual studio(>= 2019) and buildtools, then move buildtools into win_build_package
https://aka.ms/vs/17/release/vs_buildtools.exe

# HOW to COMPILE
cd xxxx/project/build
./compile_mingw.sh
```

## macos
```shell
# Install Xcode command-line tools
xcode-select –install

# Install Homebrew Package Manager which can install various development tools and software packages
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Add brew to your PATH
echo 'eval "$(/opt/homebrew/bin/brew shellenv)"' >> /Users/[your-user-name]/.zprofile
eval "$(/opt/homebrew/bin/brew shellenv)"

# install dependency libraries
brew install openssl gnu-tar rsync

# To make the apps/tools/parse/ source, you may need to install the following packages:
brew install eigen

# To make the src/ws_utils/ source, you may need to install the following packages:
brew install boost

# HOW TO COMPILE
cd /project
make build
make tarball_public
```
