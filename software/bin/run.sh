#!/bin/bash

# 设置堆栈大小为 16 MB
ulimit -s unlimited

# 启动你的程序
../build/main
