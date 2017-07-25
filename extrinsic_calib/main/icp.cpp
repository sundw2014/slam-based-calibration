/*
 * icp.cpp
 * Copyright (C) 2016 Nick.Liao <simplelife_nick@hotmail.com>
 *
 * Distributed under terms of the MIT license.
 */

#include "pointmatcher/PointMatcher.h"

#include <cassert>
#include <fstream>
#include <iostream>
#include <algorithm>

using namespace std;
using namespace PointMatcherSupport;

//typedef PointMatcher<float> PM;
//typedef PM::DataPoints DP;

using PM=PointMatcher<float>;
typedef PM::Parameters Parameters;
using DP = PM::DataPoints;


int main(int argc, const char *argv[])
{
	const char *refFile(argv[argc - 2]);
	const char *dataFile(argv[argc - 1]);

	// Load point clouds
	DP ref{ DP::load(refFile) };
	DP data{ DP::load(dataFile) };

	// Create the default ICP algorithm
	PM::ICP icp;

	if (argc == 3)
	{
		// See the implementation of setDefault() to create a custom ICP algorithm
		icp.setDefault();
	}
	else
	{
		// load YAML config
		ifstream ifs(argv[1]);
		if (!ifs.good())
		{
			cerr << "Cannot open config file ";
			icp.setDefault();
		}
		else
		{
			icp.loadFromYaml(ifs);
		}

	}

	int cloudDimension = ref.getEuclideanDim();

	if (!(cloudDimension == 2 || cloudDimension == 3))
	{
		cerr << "Invalid input point clouds dimension" << endl;
		exit(1);
	}

	// Compute the transformation to express data in ref
	PM::TransformationParameters T = icp(data, ref);
	float matchRatio = icp.errorMinimizer->getWeightedPointUsedRatio();
	cout << "match ratio: " <<  matchRatio << endl;



	// Transform data to express it in ref
	DP data_out(data);
	icp.transformations.apply(data_out, T);

	cout << endl << "------------------" << endl;

	// Structure to hold future match results
	PM::Matches matches;

	Parametrizable::Parameters params;
	params["knn"] =  toParam(1); // for Hausdorff distance, we only need the first closest point
	params["epsilon"] =  toParam(0);

	PM::Matcher* matcherHausdorff = PM::get().MatcherRegistrar.create("KDTreeMatcher", params);

	// max. distance from reading to reference
	matcherHausdorff->init(ref);
	matches = matcherHausdorff->findClosests(data_out);
	float maxDist1 = matches.getDistsQuantile(1.0);
	float maxDistRobust1 = matches.getDistsQuantile(0.85);

	// max. distance from reference to reading
	matcherHausdorff->init(data_out);
	matches = matcherHausdorff->findClosests(ref);
	float maxDist2 = matches.getDistsQuantile(1.0);
	float maxDistRobust2 = matches.getDistsQuantile(0.85);

	float haussdorffDist = std::max(maxDist1, maxDist2);
	float haussdorffQuantileDist = std::max(maxDistRobust1, maxDistRobust2);

	cout << "Haussdorff distance: " << std::sqrt(haussdorffDist) << " m" << endl;
	cout << "Haussdorff quantile distance: " << std::sqrt(haussdorffQuantileDist) <<  " m" << endl;

	// initiate the matching with unfiltered point cloud
	icp.matcher->init(ref);

	// extract closest points
	matches = icp.matcher->findClosests(data_out);

	// weight paired points
	const PM::OutlierWeights outlierWeights = icp.outlierFilters.compute(data_out, ref, matches);

	// generate tuples of matched points and remove pairs with zero weight
	const PM::ErrorMinimizer::ErrorElements matchedPoints( data_out, ref, outlierWeights, matches);

	// extract relevant information for convenience
	const int dim = matchedPoints.reading.getEuclideanDim();
	const int nbMatchedPoints = matchedPoints.reading.getNbPoints();
	const PM::Matrix matchedRead = matchedPoints.reading.features.topRows(dim);
	const PM::Matrix matchedRef = matchedPoints.reference.features.topRows(dim);

	// compute mean distance
	const PM::Matrix dist = (matchedRead - matchedRef).colwise().norm(); // replace that by squaredNorm() to save computation time
	const float meanDist = dist.sum() / nbMatchedPoints;
	cout << "Robust mean distance: " << meanDist << " m" << endl;
	// END demo
	cout << "------------------" << endl << endl;
	cout << "ICP transformation:" << endl << T << endl;

	return 0;
}
