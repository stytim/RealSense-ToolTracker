#pragma once

#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>

class IRToolKalmanFilter
{
public:
	IRToolKalmanFilter() {
		m_filter = cv::KalmanFilter(6, 3, 0, CV_32F);

	}
	IRToolKalmanFilter(float measurementNoise, float positionNoise, float velocityNoise) {
		m_filter = cv::KalmanFilter(6, 3, 0, CV_32F);
		m_fPositionNoise = positionNoise;
		m_fVelocityNoise = velocityNoise;
		m_fMeasurementNoise = measurementNoise;
	}
	cv::Vec3f FilterData(cv::Vec3f value) {
		if (!m_bInitialized) {
			InitializeFilter(value);
		}
		cv::Mat prediction = m_filter.predict();
		measurement = cv::Mat(value).reshape(1, 3);
		cv::Mat correction = m_filter.correct(measurement);

		return cv::Vec3f(correction.at<float>(0, 0), correction.at<float>(1, 0), correction.at<float>(2, 0));
	}
private:
	void InitializeFilter(cv::Vec3f value) {
		// Initialize the state transition matrix (A)
		cv::Mat A = cv::Mat::eye(6, 6, CV_32F);
		A.at<float>(0, 3) = 1.f; // x += vx*dt
		A.at<float>(1, 4) = 1.f; // y += vy*dt
		A.at<float>(2, 5) = 1.f; // z += vz*dt
		m_filter.transitionMatrix = A;

		// Initialize the measurement matrix (H)
		cv::Mat H = cv::Mat::zeros(3, 6, CV_32F);
		H.at<float>(0, 0) = 1.f;// x
		H.at<float>(1, 1) = 1.f;// y
		H.at<float>(2, 2) = 1.f;// z
		m_filter.measurementMatrix = H;

		// Initialize the process noise covariance matrix (Q)
		cv::Mat Q = cv::Mat::zeros(6, 6, CV_32F);
		Q.at<float>(0, 0) = Q.at<float>(1, 1) = Q.at<float>(2, 2) = m_fPositionNoise; // position noise
		Q.at<float>(3, 3) = Q.at<float>(4, 4) = Q.at<float>(5, 5) = m_fVelocityNoise; // velocity noise
		m_filter.processNoiseCov = Q;

		// Initialize the measurement noise covariance matrix (R)
		cv::Mat R = cv::Mat::eye(3, 3, CV_32F) * m_fMeasurementNoise; // measurement noise
		m_filter.measurementNoiseCov = R;

		// Initialize the state estimate (x) and the error covariance matrix (P)
		cv::Mat x = cv::Mat::zeros(6, 1, CV_32F); // initial state is all zeros
		cv::Mat P = cv::Mat::eye(6, 6, CV_32F); // initial error covariance is identity matrix
		m_filter.statePre = x;
		m_filter.errorCovPost = P;

		m_bInitialized = true;
		return;
	}

	cv::Mat measurement = cv::Mat(1, 3, CV_32F);
	//cv::Mat_<float> measurement(3, 1);
	bool m_bInitialized = false;
	cv::KalmanFilter m_filter;
	float m_fMeasurementNoise = 1;// 1e-1; - measurements are usually within 3mm of position (3^2=9)
	float m_fPositionNoise = 1e-4;//1e-4; - prediction is usually within .1mm of position (.1^2 = 1e-2)
	float m_fVelocityNoise = 3;// 1e-2; - velocity is probably similar

};