OBJS = rrt.cc

CC = g++ -std=c++11

LIBS = rrt.hh

# -Wall will turn on all standard warnings
COMPILER_FLAGS = -Wall

OUTPUT_NAME = rrt

all:
	$(CC) $(OBJS) $(LIBS) -o $(OUTPUT_NAME)

test:
	$(CC) rrt_test.cc $(LIBS) -o rrt_test