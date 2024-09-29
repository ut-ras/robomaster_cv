CXX=nvcc
FLAGS=-lcublas -c -Xcompiler -fPIC 
FLAGS_EXEC = -lcublas
all: clean armor-plate bounding-box object-log kalman shared

object-log-exec.o: ObjectLog.cu ObjectLog.h ArmorPlate.cu ArmorPlate.h BoundingBox.cu BoundingBox.h Kalman.cu Kalman.h
	$(CXX) $(FLAGS_EXEC) ObjectLog.cu ArmorPlate.cu BoundingBox.cu Kalman.cu -o object-log-exec.o
object-log-alone: ObjectLog.cu ObjectLog.h
	$(CXX) $(FLAGS) ObjectLog.cu -o out.o
armor-plate: ArmorPlate.cu ArmorPlate.h BoundingBox.cu BoundingBox.h
	$(CXX) $(FLAGS) ArmorPlate.cu -o armor-plate.o
bounding-box: BoundingBox.cu BoundingBox.h
	$(CXX) $(FLAGS) BoundingBox.cu -o bounding-box.o
object-log: ObjectLog.cu ObjectLog.h
	$(CXX) $(FLAGS) ObjectLog.cu -o object-log.o
kalman: Kalman.cu Kalman.h
	$(CXX) $(FLAGS) Kalman.cu -o kalman.o
shared: ObjectLog.cu ObjectLog.h ArmorPlate.cu ArmorPlate.h BoundingBox.cu BoundingBox.h Kalman.cu Kalman.h
	$(CXX) -lcublas -G armor-plate.o bounding-box.o object-log.o kalman.o -shared -o libobjectlog.so
clean:
	rm -f armor-plate.o
	rm -f bounding-box.o
	rm -f object-log.o
	rm -f out.o
	rm -f kalman.o
	rm -f BoundingBox.o
	
