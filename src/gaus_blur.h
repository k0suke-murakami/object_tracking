//
// Created by kosuke on 11/26/17.
//




#ifndef OBJECT_TRACKING_GAUS_BLUR_H
#define OBJECT_TRACKING_BLUR_H

#include <vector>
#include <array>
#include "ground_removal.h"

class GaussSmooth{
private:
	double gauss(const double sigma, const double x);

	std::vector<double> gaussKernel(const int samples, const double sigma);
public:
	GaussSmooth();
	void gaussSmoothen(std::array<Cell, numBin_>& values, const double sigma, const int samples);
};

#endif //OBJECT_TRACKING_GAUS_BLUR_H
