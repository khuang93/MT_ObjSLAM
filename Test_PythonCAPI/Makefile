CC=g++

# CXXFLAGS= -std=c++11 #-g
MRCNNPY35= /home/khuang/mrcnn/include/python3.5m
CXXFLAGS = -I $(MRCNNPY35)

LINKERFLAGS= -L/home/khuang/mrcnn/lib/python3.5/config-3.5m-x86_64-linux-gnu -lpython3.5

OBJ= test_python_capi

OBJ_CPP= $(OBJ).cpp 
OBJ_OUT= $(OBJ).out
# .DEFAULT_GOAL := all

#.PHONY := clean

all: 
	$(CC) -c $(OBJ_CPP) $(CXXFLAGS) -o $(OBJ_OUT) $(LINKERFLAGS)

clean:
	rm -rf $(OBJ_OUT)