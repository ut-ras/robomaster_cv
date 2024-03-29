CXX=nvcc
FLAGS=-lcublas
all: clean object-log 

armor-plate: ArmorPlate.cu ArmorPlate.h BoundingBox.cpp BoundingBox.h
	$(CXX) $(FLAGS) ArmorPlate.cu BoundingBox.cpp -o armor-plate.o
bounding-box: BoundingBox.cpp BoundingBox.h
	$(CXX) $(FLAGS) BoundingBox.cpp -o bounding-box.o
object-log: ObjectLog.cpp ObjectLog.h
	$(CXX) $(FLAGS) ObjectLog.cpp ArmorPlate.cu BoundingBox.cpp -o object-log.o
kalman: Kalman.cu Kalman.h
	$(CXX) $(FLAGS) Kalman.cu -o kalman.o



clean:
	rm -f armor-plate.o
	rm -f bounding-box.o
	rm -f object-log.o
	rm -f out.o
	rm -f kalman.o
	
