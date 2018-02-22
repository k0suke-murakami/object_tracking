//
// Created by kosuke on 11/26/17.
//
//#pragma once
//https://gist.github.com/agrafix/56592043c43c8801f40ab7667b9e7f0e
// modified some parts


#include <iostream>
#include <cmath>
#include <vector>
#include <array>
#include <assert.h>

#include "ground_removal.h"
#include "gaus_blur.h"

using namespace std;


GaussSmooth::GaussSmooth(){

}

double GaussSmooth::gauss(const double sigma, const double x) {
    double expVal = -1 * (pow(x, 2) / pow(2 * sigma, 2));
    double divider = sqrt(2 * M_PI * pow(sigma, 2));
    return (1 / divider) * exp(expVal);
}

std::vector<double> GaussSmooth::gaussKernel(const int samples, const double sigma) {
    std::vector<double> kernel(samples);
    double mean = samples/2;
    double sum = 0.0; // For accumulating the kernel values
    for (int x = 0; x < samples; ++x) {
        kernel[x] = exp( -0.5 * (pow((x-mean)/sigma, 2.0)))/(2 * M_PI * sigma * sigma);
        // Accumulate the kernel values
        sum += kernel[x];
    }

    // Normalize the kernel
    for (int x = 0; x < samples; ++x){
        kernel[x] /= sum;
    }
    
    assert(kernel.size() == samples);

    return kernel;
}

void GaussSmooth::gaussSmoothen(std::array<Cell, numBin_>& values, const double sigma, const int samples) {
    std::vector<double> kernel = gaussKernel(samples, sigma);
    int sampleSide = samples / 2;
    unsigned long ubound = values.size();
    // applying gaussian kernel with zero padding
    for (long i = 0; i < ubound; i++) {
        double smoothed = 0;
        for (long j = i - sampleSide; j <= i + sampleSide; j++) {
            if (j >= 0 && j < ubound) {
                int sampleWeightIndex = sampleSide + (j - i);
                smoothed += kernel[sampleWeightIndex] * values[j].getHeight();
            }
        }
        // std::cout << " V: " << values[i].getHeight() << " SM: " << smoothed << std::endl;
        values[i].updateSmoothed(smoothed);
    }
}



