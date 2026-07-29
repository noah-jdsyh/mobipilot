# How to build and run on windows

> This document is a manual for build and run falcon sdk on windows

## Environment

> we can easily crossbuild on linux by Mingw-w64, see build/bitbucket-pipelines-build-mingw64.bash

- Windows 11

## Build Dependencies

- MSYS2(Mingw-w64)
  - Boost
  - Openssl
  - Eigen

## Build Procedure

### Install Dependencies

> All dependencies will be installed in this section, if you have already installed you can skip this section

#### MSYS2

> Refer to https://www.msys2.org/, don't forget to run **pacman -Syu** to update

we choose **MINGW64** env as default, for more info please refer to https://www.msys2.org/docs/environments/

Now, run this command in msys2 terminal(type "msys" in Windows search bar)

```bash
pacman -S --needed base-devel mingw-w64-x86_64-toolchain
```

#### Falcon SDK dependencies

> boost, openssl are required

```bash
pacman -Ss boost
pacman -S mingw-w64-x86_64-boost
pacman -Ss openssl
pacman -S mingw-w64-x86_64-openssl
```

### Clone Code

> It is recommended to install git under windows (check the symlinks option), and you need to open the developer mode and run git bash as an administrator
>
> Refer to: https://github.com/git-for-windows/git/wiki/Symbolic-Links

### Compile SDK

> Victory is here, we just need one step!

Execute the following command

```bash
./build/build-win-mingw64.bash
```

## Debug in vscode

### launch.json

```c
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "C/C++: g++.exe build and debug active file",
            "type": "cppdbg",
            "request": "launch",
            "program": "***path***\\pcs\\inno_pc_server.exe",
            "args": [
                "--lidar-ip",
                "172.168.1.10",
                "--lidar-port",
                "8002"
            ],
            "stopAtEntry": false,
            "cwd": "${fileDirname}",
            "environment": [],
            "externalConsole": false,
            "MIMode": "gdb",
            "miDebuggerPath": "C:\\msys64\\mingw64\\bin\\gdb.exe",
            "setupCommands": [
                {
                    "description": "Enable pretty-printing for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                }
            ],
        }
    ]
}
```

### *MSYS2 terminal in vscode

> Shift + Control + P -> Open User Setting(JSON)

#### Setting.json

```c
    "terminal.integrated.defaultProfile.windows": "mysys2-mingw64",
    "terminal.integrated.profiles.windows": {
        "mysys2-mingw64": {
            "path": "cmd.exe",
            "args": [
                "/c",
                "C:\\msys64\\msys2_shell.cmd -defterm -mingw64 -no-start -here"
            ]
        }
    },
```
