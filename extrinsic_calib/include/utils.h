/*
 * utils.h
 * Copyright (C) 2016 Nick.Liao <simplelife_nick@hotmail.com>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <array>

#include <Eigen/Dense>
#include <Eigen/Geometry>

class utils
{
public:
	static double quaternion_error(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2,
								   bool is_normlized= true);
	template <typename T>
	struct Type2Type
	{
		typedef T type;
	};

	inline static int parese_string(std::string& s, Type2Type<int> )
	{
		return std::stoi(s);
	}

	inline static double parese_string(std::string& s, Type2Type<double> )
	{
		return std::stod(s);
	}

	template<typename  T>
	static bool csv2vector(const std::string& fn, std::vector<std::vector<T>>& vec,
						   std::size_t cols=4, char sep=' ')
	{
		std::ifstream f(fn);

		if(!f)
		{
			std::cerr<< __FUNCTION__ << " fail to open "<< fn << std::endl;
			f.close();
			return false;
		}

		std::string line;
		std::vector<T> row;
		row.reserve(cols);

		while(std::getline(f, line))
		{
			std::istringstream lines(line);
			std::string s;

			while(std::getline(lines, s, sep))
			{
				row.push_back(parese_string( s, Type2Type<T>() ));
			}

			vec.push_back(row);
			row.clear();
		}

		f.close();
		return true;
	}

	inline static double point_distance(std::array<uint16_t,2>& a, std::array<uint16_t,2>& b )
	{
		double x = a[0]-b[0];
		double y = a[1]-b[1];
		return std::sqrt(x*x+y*y);
	}

	inline static uint32_t index_around(int64_t x, uint32_t max)
	{
		return x>max?max:(x<0?0:x);
	}


};




#endif /* !UTILS_H */
