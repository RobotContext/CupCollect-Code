INCLUDES = -I. -I /usr/boost_1_51_0/
CXX= g++
CXXFLAGS = -g ${INCLUDES}
OBJECTS = Image.o PPMLoader.o OfflineCellPlanner.o OfflineReachableMap.o run.o map.o coverage.o cupScanner.o road_map.o
run : ${OBJECTS}
	${CXX} ${CFLAGS} ${OBJECTS} -o run

%.o : %.cpp
	${CXX} ${CXXFLAGS} -c $<

.PHONY clean:
	rm -f count *.o *.~
