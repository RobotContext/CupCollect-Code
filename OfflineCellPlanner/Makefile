INCLUDES = -I. -I ~/.src/boost_1_51_0/
CXX= g++
CXXFLAGS = -g ${INCLUDES}
OBJECTS = Image.o PPMLoader.o OfflineCellPlanner.o OfflineReachableMap.o run.o map.o
run : ${OBJECTS}
	${CXX} ${CFLAGS} ${OBJECTS} -o run

%.o : %.cpp
	${CXX} ${CXXFLAGS} -c $<

.PHONY clean:
	rm -f count *.o *.~
