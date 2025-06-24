#include "cv_bridge/cv_bridge.h"
#include "opencv2/video/tracking.hpp"
#include <cfloat>
#include <set>
#include <vector>
#include <iostream>

#define STATE_NUM 4  // [x, y, vx, vy]
#define MEASURE_NUM 2  // [x, y]

using namespace std;

struct TrackerState {
    float x;
    float y;

    cv::Mat toMat() const {
        cv::Mat mat = cv::Mat::zeros(MEASURE_NUM, 1, CV_32F);
        mat.at<float>(0, 0) = x;
        mat.at<float>(1, 0) = y;
        return mat;
    }

    static TrackerState fromMat(const cv::Mat& mat) {
        TrackerState s;
        s.x = mat.at<float>(0, 0);
        s.y = mat.at<float>(1, 0);
        return s;
    }
};

class Tracker {
public:
    int m_time_since_update;
    int m_hits;
    int m_hit_streak;
    int m_id;

    Tracker(const TrackerState& state);
    TrackerState predict();
    void update(const TrackerState& state);
    TrackerState getState() const;

private:
    static int kf_count;
    cv::KalmanFilter kf;
};

int Tracker::kf_count = 0;

Tracker::Tracker(const TrackerState& state) {
    m_time_since_update = 0;
    m_hits = 0;
    m_hit_streak = 0;
    m_id = ++kf_count;

    kf = cv::KalmanFilter(STATE_NUM, MEASURE_NUM, 0);
    
    // state transition [x, y, vx, vy]
    kf.transitionMatrix = (cv::Mat_<float>(STATE_NUM, STATE_NUM) <<
                           1, 0, 1, 0,
                           0, 1, 0, 1,
                           0, 0, 1, 0,
                           0, 0, 0, 1);
    
    cv::setIdentity(kf.measurementMatrix);
    cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-2));
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-1));
    cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1));

    // initialize state
    kf.statePost.at<float>(0, 0) = state.x;
    kf.statePost.at<float>(1, 0) = state.y;
    kf.statePost.at<float>(2, 0) = 0;
    kf.statePost.at<float>(3, 0) = 0;
}

TrackerState Tracker::predict() {
    if (m_time_since_update > 0)
        m_hit_streak = 0;
    m_time_since_update++;

    cv::Mat pred = kf.predict();
    return TrackerState::fromMat(pred);
}

void Tracker::update(const TrackerState& state) {
    m_time_since_update = 0;
    m_hits++;
    m_hit_streak++;

    cv::Mat measurement = cv::Mat::zeros(MEASURE_NUM, 1, CV_32F);
    measurement.at<float>(0, 0) = state.x;
    measurement.at<float>(1, 0) = state.y;
    kf.correct(measurement);
}

TrackerState Tracker::getState() const {
    return TrackerState::fromMat(kf.statePost);
}

struct SortRect {
    int id;
    float centerX;
    float centerY;

    TrackerState toTrackerState() const {
        TrackerState s;
        s.x = centerX;
        s.y = centerY;
        return s;
    }

    void fromTrackerState(const TrackerState& s) {
        centerX = s.x;
        centerY = s.y;
    }
};

class AssignmentSolver {
public:
    AssignmentSolver() {}
    ~AssignmentSolver() {}
    double Solve(vector<vector<double>>& DistMatrix, vector<int>& Assignment);
private:
    void assignmentoptimal(int* assignment, double* cost, double* distMatrix, int nRows, int nCols);
    void buildassignmentvector(int* assignment, bool* starMatrix, int nRows, int nCols);
    void computeassignmentcost(int* assignment, double* cost, double* distMatrix, int nRows);
    void step2a(...);
    void step2b(...);
    void step3(...);
    void step4(...);
    void step5(...);
};

class Sort {
public:
    Sort(int maxAge = 2, int minHits = 3, double distThreshold = 100.0)
        : maxAge(maxAge), minHits(minHits), distThreshold(distThreshold) {}

    vector<SortRect> update(const vector<SortRect>& detections);

private:
    AssignmentSolver hungarian;
    int maxAge;
    int minHits;
    double distThreshold;
    vector<Tracker> trackers;
};

vector<SortRect> Sort::update(const vector<SortRect>& detections) {
    // create trackers if empty
    if (trackers.empty()) {
        for (const auto& det : detections) {
            trackers.emplace_back(det.toTrackerState());
        }
        return {};
    }

    // predict
    vector<SortRect> preds;
    for (auto& tr : trackers) {
        SortRect r; r.fromTrackerState(tr.predict()); r.id = -1;
        preds.push_back(r);
    }

    // build cost matrix (squared Euclid)
    int N = preds.size(), M = detections.size();
    vector<vector<double>> cost(N, vector<double>(M));
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < M; ++j) {
            double dx = preds[i].centerX - detections[j].centerX;
            double dy = preds[i].centerY - detections[j].centerY;
            cost[i][j] = dx*dx + dy*dy;
        }
    }

    // solve assignment
    vector<int> assignment;
    hungarian.Solve(cost, assignment);

    set<int> unmatchedDet;
    for (int j = 0; j < M; ++j) unmatchedDet.insert(j);
    vector<pair<int,int>> matched;

    for (int i = 0; i < N; ++i) {
        int j = assignment[i];
        if (j >= 0 && j < M && cost[i][j] <= distThreshold) {
            matched.emplace_back(i, j);
            unmatchedDet.erase(j);
        }
    }

    // update matched
    for (auto& p : matched) {
        trackers[p.first].update(detections[p.second].toTrackerState());
    }
    // create new trackers for unmatched
    for (int j : unmatchedDet) {
        trackers.emplace_back(detections[j].toTrackerState());
    }

    // collect results and prune
    vector<SortRect> output;
    for (auto it = trackers.begin(); it != trackers.end(); ) {
        if (it->m_time_since_update > maxAge) {
            it = trackers.erase(it);
        } else {
            if (it->m_hit_streak >= minHits && it->m_time_since_update == 0) {
                SortRect r; r.fromTrackerState(it->getState()); r.id = it->m_id;
                output.push_back(r);
            }
            ++it;
        }
    }
    return output;
}
