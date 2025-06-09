#include "cv_bridge/cv_bridge.h"
#include "opencv2/video/tracking.hpp"
#include <cfloat>
#include <set>
#include <vector>
#include <iostream>


#define STATE_NUM 7
#define MEASURE_NUM 4

using namespace std; 

/*
* -----------------------------------------------
* |                                             |
* |                                             |
* |            TrackerState.hpp                 |
* |                                             |
* -----------------------------------------------
*/
struct TrackerState {
	float centerX;
	float centerY;
	float area;
	float aspectRatio;

	cv::Mat toMat(void);
	void fromMat(cv::Mat mat);

};

/*
* -----------------------------------------------
* |                                             |
* |                                             |
* |               Tracker.hpp                   |
* |                                             |
* -----------------------------------------------
*/
class Tracker {

    public:

        int m_time_since_update;
        int m_hits;
        int m_hit_streak;
        int m_id;

        Tracker(TrackerState state);

        TrackerState predict(void);
        void update(TrackerState state);
        TrackerState getState();


    private:

        static int kf_count;

        cv::KalmanFilter kf;

};

/*
* -----------------------------------------------
* |                                             |
* |                                             |
* |               SortRect.hpp                  |
* |                                             |
* -----------------------------------------------
*/
struct SortRect {

    int id;
    float centerX;
    float centerY;
    float width;
    float height;

    TrackerState toTrackerState(void);
    void fromTrackerState(TrackerState state);
};

/*
* -----------------------------------------------
* |                                             |
* |                                             |
* |               Sort.hpp                      |
* |                                             |
* -----------------------------------------------
*/
class Sort {

    public:

        Sort(int maxAge, int minHits, float iouThreshold);

        std::vector<SortRect> update(std::vector<SortRect> rects);


    private:

        HungarianAlgorithm HungAlgo;

        int maxAge;
        int minHits;
        float iouThreshold;

        std::vector<Tracker> trackers;

        float iou(SortRect rect1, SortRect rect2);

};

/*
* -----------------------------------------------
* |                                             |
* |                                             |
* |             Hungarian.hpp                   |
* |                                             |
* -----------------------------------------------
*/
class HungarianAlgorithm
{
public:
	HungarianAlgorithm();
	~HungarianAlgorithm();
	double Solve(vector<vector<double>>& DistMatrix, vector<int>& Assignment);

private:
	void assignmentoptimal(int *assignment, double *cost, double *distMatrix, int nOfRows, int nOfColumns);
	void buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns);
	void computeassignmentcost(int *assignment, double *cost, double *distMatrix, int nOfRows);
	void step2a(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
	void step2b(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
	void step3(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
	void step4(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col);
	void step5(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
};


/*
* -----------------------------------------------
* |                                             |
* |                                             |
* |               Tracker.cpp                   |
* |                                             |
* -----------------------------------------------
*/

int Tracker::kf_count = 0;


Tracker::Tracker(TrackerState state) {

    kf_count++;

    // Initialize variables
    m_time_since_update = 0;
    m_hits = 0;
    m_hit_streak = 0;
    m_id = kf_count;


    // Initialize kalman filter
    kf = cv::KalmanFilter(STATE_NUM, MEASURE_NUM, 0);

    kf.transitionMatrix = (cv::Mat_<float>(STATE_NUM, STATE_NUM) <<
        1, 0, 0, 0, 1, 0, 0,
        0, 1, 0, 0, 0, 1, 0,
        0, 0, 1, 0, 0, 0, 1,
        0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 1
    );

    cv::setIdentity( kf.measurementMatrix );
    cv::setIdentity( kf.processNoiseCov,        cv::Scalar::all(1e-2)   );
    cv::setIdentity( kf.measurementNoiseCov,    cv::Scalar::all(1e-1)   );
    cv::setIdentity( kf.errorCovPost,           cv::Scalar::all(1)      );

    // Initialize state vector
    kf.statePost.at<float>(0, 0) = state.centerX;
    kf.statePost.at<float>(1, 0) = state.centerY;
    kf.statePost.at<float>(2, 0) = state.area;
    kf.statePost.at<float>(3, 0) = state.aspectRatio;
    
}


TrackerState Tracker::predict(void) {

    if (m_time_since_update > 0)
        m_hit_streak = 0;
    m_time_since_update += 1;

    TrackerState state;
    state.fromMat(kf.predict());

    if(state.area < 0) {
        state.area = 0;
    }
    
    return state;

}


void Tracker::update(TrackerState state) {

    m_time_since_update = 0;
    m_hits += 1;
    m_hit_streak += 1;

    cv::Mat measurement = cv::Mat::zeros(MEASURE_NUM, 1, CV_32F);
    measurement.at<float>(0, 0) = state.centerX;
    measurement.at<float>(1, 0) = state.centerY;
    measurement.at<float>(2, 0) = state.area;
    measurement.at<float>(3, 0) = state.aspectRatio;

    kf.correct(measurement);

}


TrackerState Tracker::getState(void) {

    TrackerState state;
    state.fromMat(kf.statePost);

    return state;
}


/*
* -----------------------------------------------
* |                                             |
* |                                             |
* |            TrackerState.cpp                 |
* |                                             |
* -----------------------------------------------
*/
cv::Mat TrackerState::toMat(void) {

    cv::Mat mat = cv::Mat::zeros(MEASURE_NUM, 1, CV_32F);
    mat.at<float>(0, 0) = centerX;
    mat.at<float>(1, 0) = centerY;
    mat.at<float>(2, 0) = area;
    mat.at<float>(3, 0) = aspectRatio;

    return mat;
}

void TrackerState::fromMat(cv::Mat mat) {

    centerX     = mat.at<float>(0, 0);
    centerY     = mat.at<float>(1, 0);
    area        = mat.at<float>(2, 0);
    aspectRatio = mat.at<float>(3, 0);
    
}


/*
* -----------------------------------------------
* |                                             |
* |                                             |
* |              SortRect.cpp                   |
* |                                             |
* -----------------------------------------------
*/
TrackerState SortRect::toTrackerState(void) {

    TrackerState state;
    state.centerX = centerX;
    state.centerY = centerY;
    state.area = width * height;
    state.aspectRatio = width / height;
    
    return state;
}

void SortRect::fromTrackerState(TrackerState state) {

    centerX = state.centerX;
    centerY = state.centerY;

    if(state.area > 0) {
        width = sqrt(state.area * state.aspectRatio);
        height = state.area / width;
    }
    else {
        width = 0;
        height = 0;
    }

}


/*
* -----------------------------------------------
* |                                             |
* |                                             |
* |               Sort.cpp                      |
* |                                             |
* -----------------------------------------------
*/

Sort::Sort(int maxAge = 2, int minHits = 3, float iouThreshold = 0.3) {

    this->maxAge = maxAge;
    this->minHits = minHits;
    this->iouThreshold = iouThreshold;

}


std::vector<SortRect> Sort::update(std::vector<SortRect> detections) {

    if(trackers.size() == 0) {
        
        for (int i=0; i<detections.size(); i++) {

            TrackerState state = detections[i].toTrackerState();
            Tracker tracker = Tracker(state);

            trackers.push_back(tracker);
        }

        return std::vector<SortRect>();
    }



    std::vector<SortRect> predictions;
    for(int i=0; i<trackers.size(); i++) {
        TrackerState state = trackers[i].predict();

        SortRect rect;
        rect.fromTrackerState(state);
        rect.id = 0;

        predictions.push_back(rect);
    }



    vector<vector<double>> iouMatrix;
	iouMatrix.resize(predictions.size(), vector<double>(detections.size(), 0));

    for(int i=0; i<predictions.size(); i++)
        for(int j=0; j<detections.size(); j++)
            iouMatrix[i][j] = 1 - iou(predictions[i], detections[j]);



    vector<int> assignment;

    HungAlgo.Solve(iouMatrix, assignment);



	set<int> allItems;
	set<int> matchedItems;
    set<int> unmatchedDetections;
    
    for (int i=0; i<detections.size(); i++)
        allItems.insert(i);

    for (int i=0; i<predictions.size(); i++)
        if (assignment[i] != -1)
            matchedItems.insert(assignment[i]);

    std::set_difference(
        allItems.begin(), allItems.end(),
        matchedItems.begin(), matchedItems.end(),
        insert_iterator<set<int>>(unmatchedDetections, unmatchedDetections.begin())
    );



    vector<std::pair<int, int>> matchedPairs;
    for(int i=0; i<assignment.size(); i++) {

        if(assignment[i] == -1) continue;

        if(1 - iouMatrix[i][assignment[i]] < iouThreshold)
            unmatchedDetections.insert(assignment[i]);
        else
            matchedPairs.push_back(make_pair(i, assignment[i]));
    }
    


    for(auto pair : matchedPairs) {

        int trackerIndex = pair.first;
        int detectionIndex = pair.second;

        TrackerState state = detections[detectionIndex].toTrackerState();

        trackers[trackerIndex].update(state);
    }


    
    for (auto detectionIndex : unmatchedDetections) {

        TrackerState state = detections[detectionIndex].toTrackerState();

        Tracker tracker = Tracker(state);
        trackers.push_back(tracker);
    }

    
    
    std::vector<SortRect> result;
    for (auto it = trackers.begin(); it != trackers.end();) {

        if((*it).m_time_since_update > maxAge) {

            it = trackers.erase(it);

        }
        else {

            if( (*it).m_time_since_update < 1 && (*it).m_hit_streak >= minHits ) {

                TrackerState state = (*it).getState();

                SortRect rect;
                rect.fromTrackerState(state);
                rect.id = (*it).m_id;

                result.push_back(rect);

                
            }

            it++;
        }

    }

    return result;

}


float Sort::iou(SortRect rect1, SortRect rect2) {

    float rect1_x1 = rect1.centerX - rect1.width/2;
    float rect1_y1 = rect1.centerY - rect1.height/2;
    float rect1_x2 = rect1.centerX + rect1.width/2;
    float rect1_y2 = rect1.centerY + rect1.height/2;

    float rect2_x1 = rect2.centerX - rect2.width/2;
    float rect2_y1 = rect2.centerY - rect2.height/2;
    float rect2_x2 = rect2.centerX + rect2.width/2;
    float rect2_y2 = rect2.centerY + rect2.height/2;


    float x1 = max(rect1_x1, rect2_x1);
    float y1 = max(rect1_y1, rect2_y1);
    float x2 = min(rect1_x2, rect2_x2);
    float y2 = min(rect1_y2, rect2_y2);

    float w = max(0.f, x2 - x1);
    float h = max(0.f, y2 - y1);

    float area1 = (rect1_x2 - rect1_x1) * (rect1_y2 - rect1_y1);
    float area2 = (rect2_x2 - rect2_x1) * (rect2_y2 - rect2_y1);
    float area3 = w * h;

    float iou = area3 / (area1 + area2 - area3 + DBL_EPSILON);

    return iou;
}



/*
* -----------------------------------------------
* |                                             |
* |                                             |
* |               Hungarian.cpp                      |
* |                                             |
* -----------------------------------------------
*/


HungarianAlgorithm::HungarianAlgorithm(){}
HungarianAlgorithm::~HungarianAlgorithm(){}


//********************************************************//
// A single function wrapper for solving assignment problem.
//********************************************************//
double HungarianAlgorithm::Solve(vector<vector<double>>& DistMatrix, vector<int>& Assignment)
{
	unsigned int nRows = DistMatrix.size();
	unsigned int nCols = DistMatrix[0].size();

	double *distMatrixIn = new double[nRows * nCols];
	int *assignment = new int[nRows];
	double cost = 0.0;

	// Fill in the distMatrixIn. Mind the index is "i + nRows * j".
	// Here the cost matrix of size MxN is defined as a double precision array of N*M elements. 
	// In the solving functions matrices are seen to be saved MATLAB-internally in row-order.
	// (i.e. the matrix [1 2; 3 4] will be stored as a vector [1 3 2 4], NOT [1 2 3 4]).
	for (unsigned int i = 0; i < nRows; i++)
		for (unsigned int j = 0; j < nCols; j++)
			distMatrixIn[i + nRows * j] = DistMatrix[i][j];
	
	// call solving function
	assignmentoptimal(assignment, &cost, distMatrixIn, nRows, nCols);

	Assignment.clear();
	for (unsigned int r = 0; r < nRows; r++)
		Assignment.push_back(assignment[r]);

	delete[] distMatrixIn;
	delete[] assignment;
	return cost;
}


//********************************************************//
// Solve optimal solution for assignment problem using Munkres algorithm, also known as Hungarian Algorithm.
//********************************************************//
void HungarianAlgorithm::assignmentoptimal(int *assignment, double *cost, double *distMatrixIn, int nOfRows, int nOfColumns)
{
	double *distMatrix, *distMatrixTemp, *distMatrixEnd, *columnEnd, value, minValue;
	bool *coveredColumns, *coveredRows, *starMatrix, *newStarMatrix, *primeMatrix;
	int nOfElements, minDim, row, col;

	/* initialization */
	*cost = 0;
	for (row = 0; row<nOfRows; row++)
		assignment[row] = -1;

	/* generate working copy of distance Matrix */
	/* check if all matrix elements are positive */
	nOfElements = nOfRows * nOfColumns;
	distMatrix = (double *)malloc(nOfElements * sizeof(double));
	distMatrixEnd = distMatrix + nOfElements;

	for (row = 0; row<nOfElements; row++)
	{
		value = distMatrixIn[row];
		if (value < 0)
			cerr << "All matrix elements have to be non-negative." << endl;
		distMatrix[row] = value;
	}


	/* memory allocation */
	coveredColumns = (bool *)calloc(nOfColumns, sizeof(bool));
	coveredRows = (bool *)calloc(nOfRows, sizeof(bool));
	starMatrix = (bool *)calloc(nOfElements, sizeof(bool));
	primeMatrix = (bool *)calloc(nOfElements, sizeof(bool));
	newStarMatrix = (bool *)calloc(nOfElements, sizeof(bool)); /* used in step4 */

	/* preliminary steps */
	if (nOfRows <= nOfColumns)
	{
		minDim = nOfRows;

		for (row = 0; row<nOfRows; row++)
		{
			/* find the smallest element in the row */
			distMatrixTemp = distMatrix + row;
			minValue = *distMatrixTemp;
			distMatrixTemp += nOfRows;
			while (distMatrixTemp < distMatrixEnd)
			{
				value = *distMatrixTemp;
				if (value < minValue)
					minValue = value;
				distMatrixTemp += nOfRows;
			}

			/* subtract the smallest element from each element of the row */
			distMatrixTemp = distMatrix + row;
			while (distMatrixTemp < distMatrixEnd)
			{
				*distMatrixTemp -= minValue;
				distMatrixTemp += nOfRows;
			}
		}

		/* Steps 1 and 2a */
		for (row = 0; row<nOfRows; row++)
			for (col = 0; col<nOfColumns; col++)
				if (abs(distMatrix[row + nOfRows*col]) < DBL_EPSILON)
					if (!coveredColumns[col])
					{
						starMatrix[row + nOfRows*col] = true;
						coveredColumns[col] = true;
						break;
					}
	}
	else /* if(nOfRows > nOfColumns) */
	{
		minDim = nOfColumns;

		for (col = 0; col<nOfColumns; col++)
		{
			/* find the smallest element in the column */
			distMatrixTemp = distMatrix + nOfRows*col;
			columnEnd = distMatrixTemp + nOfRows;

			minValue = *distMatrixTemp++;
			while (distMatrixTemp < columnEnd)
			{
				value = *distMatrixTemp++;
				if (value < minValue)
					minValue = value;
			}

			/* subtract the smallest element from each element of the column */
			distMatrixTemp = distMatrix + nOfRows*col;
			while (distMatrixTemp < columnEnd)
				*distMatrixTemp++ -= minValue;
		}

		/* Steps 1 and 2a */
		for (col = 0; col<nOfColumns; col++)
			for (row = 0; row<nOfRows; row++)
				if (abs(distMatrix[row + nOfRows*col]) < DBL_EPSILON)
					if (!coveredRows[row])
					{
						starMatrix[row + nOfRows*col] = true;
						coveredColumns[col] = true;
						coveredRows[row] = true;
						break;
					}
		for (row = 0; row<nOfRows; row++)
			coveredRows[row] = false;

	}

	/* move to step 2b */
	step2b(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);

	/* compute cost and remove invalid assignments */
	computeassignmentcost(assignment, cost, distMatrixIn, nOfRows);

	/* free allocated memory */
	free(distMatrix);
	free(coveredColumns);
	free(coveredRows);
	free(starMatrix);
	free(primeMatrix);
	free(newStarMatrix);

	return;
}

/********************************************************/
void HungarianAlgorithm::buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns)
{
	int row, col;

	for (row = 0; row<nOfRows; row++)
		for (col = 0; col<nOfColumns; col++)
			if (starMatrix[row + nOfRows*col])
			{
#ifdef ONE_INDEXING
				assignment[row] = col + 1; /* MATLAB-Indexing */
#else
				assignment[row] = col;
#endif
				break;
			}
}

/********************************************************/
void HungarianAlgorithm::computeassignmentcost(int *assignment, double *cost, double *distMatrix, int nOfRows)
{
	int row, col;

	for (row = 0; row<nOfRows; row++)
	{
		col = assignment[row];
		if (col >= 0)
			*cost += distMatrix[row + nOfRows*col];
	}
}

/********************************************************/
void HungarianAlgorithm::step2a(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
	bool *starMatrixTemp, *columnEnd;
	int col;

	/* cover every column containing a starred zero */
	for (col = 0; col<nOfColumns; col++)
	{
		starMatrixTemp = starMatrix + nOfRows*col;
		columnEnd = starMatrixTemp + nOfRows;
		while (starMatrixTemp < columnEnd){
			if (*starMatrixTemp++)
			{
				coveredColumns[col] = true;
				break;
			}
		}
	}

	/* move to step 3 */
	step2b(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

/********************************************************/
void HungarianAlgorithm::step2b(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
	int col, nOfCoveredColumns;

	/* count covered columns */
	nOfCoveredColumns = 0;
	for (col = 0; col<nOfColumns; col++)
		if (coveredColumns[col])
			nOfCoveredColumns++;

	if (nOfCoveredColumns == minDim)
	{
		/* algorithm finished */
		buildassignmentvector(assignment, starMatrix, nOfRows, nOfColumns);
	}
	else
	{
		/* move to step 3 */
		step3(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
	}

}

/********************************************************/
void HungarianAlgorithm::step3(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
	bool zerosFound;
	int row, col, starCol;

	zerosFound = true;
	while (zerosFound)
	{
		zerosFound = false;
		for (col = 0; col<nOfColumns; col++)
			if (!coveredColumns[col])
				for (row = 0; row<nOfRows; row++)
					if ((!coveredRows[row]) && (abs(distMatrix[row + nOfRows*col]) < DBL_EPSILON))
					{
						/* prime zero */
						primeMatrix[row + nOfRows*col] = true;

						/* find starred zero in current row */
						for (starCol = 0; starCol<nOfColumns; starCol++)
							if (starMatrix[row + nOfRows*starCol])
								break;

						if (starCol == nOfColumns) /* no starred zero found */
						{
							/* move to step 4 */
							step4(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim, row, col);
							return;
						}
						else
						{
							coveredRows[row] = true;
							coveredColumns[starCol] = false;
							zerosFound = true;
							break;
						}
					}
	}

	/* move to step 5 */
	step5(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

/********************************************************/
void HungarianAlgorithm::step4(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col)
{
	int n, starRow, starCol, primeRow, primeCol;
	int nOfElements = nOfRows*nOfColumns;

	/* generate temporary copy of starMatrix */
	for (n = 0; n<nOfElements; n++)
		newStarMatrix[n] = starMatrix[n];

	/* star current zero */
	newStarMatrix[row + nOfRows*col] = true;

	/* find starred zero in current column */
	starCol = col;
	for (starRow = 0; starRow<nOfRows; starRow++)
		if (starMatrix[starRow + nOfRows*starCol])
			break;

	while (starRow<nOfRows)
	{
		/* unstar the starred zero */
		newStarMatrix[starRow + nOfRows*starCol] = false;

		/* find primed zero in current row */
		primeRow = starRow;
		for (primeCol = 0; primeCol<nOfColumns; primeCol++)
			if (primeMatrix[primeRow + nOfRows*primeCol])
				break;

		/* star the primed zero */
		newStarMatrix[primeRow + nOfRows*primeCol] = true;

		/* find starred zero in current column */
		starCol = primeCol;
		for (starRow = 0; starRow<nOfRows; starRow++)
			if (starMatrix[starRow + nOfRows*starCol])
				break;
	}

	/* use temporary copy as new starMatrix */
	/* delete all primes, uncover all rows */
	for (n = 0; n<nOfElements; n++)
	{
		primeMatrix[n] = false;
		starMatrix[n] = newStarMatrix[n];
	}
	for (n = 0; n<nOfRows; n++)
		coveredRows[n] = false;

	/* move to step 2a */
	step2a(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

/********************************************************/
void HungarianAlgorithm::step5(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
	double h, value;
	int row, col;

	/* find smallest uncovered element h */
	h = DBL_MAX;
	for (row = 0; row<nOfRows; row++)
		if (!coveredRows[row])
			for (col = 0; col<nOfColumns; col++)
				if (!coveredColumns[col])
				{
					value = distMatrix[row + nOfRows*col];
					if (value < h)
						h = value;
				}

	/* add h to each covered row */
	for (row = 0; row<nOfRows; row++)
		if (coveredRows[row])
			for (col = 0; col<nOfColumns; col++)
				distMatrix[row + nOfRows*col] += h;

	/* subtract h from each uncovered column */
	for (col = 0; col<nOfColumns; col++)
		if (!coveredColumns[col])
			for (row = 0; row<nOfRows; row++)
				distMatrix[row + nOfRows*col] -= h;

	/* move to step 3 */
	step3(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}