CXX=nvcc
FLAGS=-lcublas
all: clean object-log 

armor-plate: ArmorPlate.cu ArmorPlate.h BoundingBox.cu BoundingBox.h
	$(CXX) $(FLAGS) ArmorPlate.cu BoundingBox.cu -o armor-plate.o
bounding-box: BoundingBox.cu BoundingBox.h
	$(CXX) $(FLAGS) BoundingBox.cu -o bounding-box.o
object-log: ObjectLog.cu ObjectLog.h
	$(CXX) $(FLAGS) ObjectLog.cu ArmorPlate.cu BoundingBox.cu -o object-log.o
kalman: Kalman.cu Kalman.h
	$(CXX) $(FLAGS) Kalman.cu -o kalman.o

clean:
	rm -f armor-plate.o
	rm -f bounding-box.o
	rm -f object-log.o
	rm -f out.o
	rm -f kalman.o
	
