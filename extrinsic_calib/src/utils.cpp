/*
 * utils.cpp
 * Copyright (C) 2016 Nick.Liao <simplelife_nick@hotmail.com>
 *
 * Distributed under terms of the MIT license.
 */

#include "utils.h"

#include <cmath>
#include <ctime>



double utils::quaternion_error(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2, bool is_normlized)
{
	Eigen::Vector4d qv1,qv2;

	qv1(0)= q1.w();
	qv1.tail(3) = q1.vec();

	qv2(0)= q2.w();
	qv2.tail(3) = q2.vec();

	if(!is_normlized)
	{
		qv1.normalize();
		qv2.normalize();
	}

	return std::acos( std::abs( qv1.transpose()*qv2 ) );
}



