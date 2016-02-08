/*
 * valueToColor.hpp
 *
 *  Created on: Jan 29, 2016
 *      Author: karrerm
 */

#ifndef INCLUDE_FASTFUSION_NODE_VALUETOCOLOR_HPP_
#define INCLUDE_FASTFUSION_NODE_VALUETOCOLOR_HPP_

#include <cmath>


typedef struct color_{
	unsigned char r;
	unsigned char g;
	unsigned char b;
}color;


class ValueToColor{
public:
	ValueToColor(double min, double max){
		if (min < max) {
			min_ = min;
			max_ = max;
		}else if (max < min){
			min_ = max;
			max_ = min;
		} else {
			min_ = 0.0;
			max_ = 1.0;
		}
		colorScale_ = 2.0/(max_-min_);
	};
	~ValueToColor(){};

	//-- Set the minimal and maximum values
	void setMinMax(double min, double max){
		if (min < max) {
			min_ = min;
			max_ = max;
		}else if (max < min){
			min_ = max;
			max_ = min;
		} else {
			min_ = 0.0;
			max_ = 1.0;
		}
	}

	//-- Compute the rgb values and return them as color struct
	color compute(const double inValue) {
		double value = inValue;
		if (inValue < min_) {
			value = min_;
		} else if (inValue > max_){
			value = max_;
		}
		value = (value - min_ )*colorScale_ - 1.0;
		color rgb;
		double rd = colorMapRed(value);
		double gd = colorMapGreen(value);
		double bd = colorMapBlue(value);
		rgb.r = (unsigned char)std::round(rd*255.0);
		rgb.g = (unsigned char)std::round(gd*255.0);
		rgb.b = (unsigned char)std::round(bd*255.0);
		return rgb;
	}
private:
	//-- Interpolate values
	double interpolate(double val, double y0, double x0, double y1, double x1){
		return (val-x0)*(y1-y0)/(x1-x0) + y0;
	};

	//-- Compute Color component
	double colorMapBase(double val){
		if ( val <= -0.75 ) return 0;
		else if ( val <= -0.25 ) return interpolate( val, 0.0, -0.75, 1.0, -0.25 );
		else if ( val <= 0.25 ) return 1.0;
		else if ( val <= 0.75 ) return interpolate( val, 1.0, 0.25, 0.0, 0.75 );
		else return 0.0;
	};

	//-- Compute Red value
	double colorMapRed(double value){
		return colorMapBase(value - 0.5 );
	};

	//-- Compute Green Value
	double colorMapGreen(double value){
		return colorMapBase(value);
	};

	//-- Compute Blue Value
	double colorMapBlue(double value){
		return colorMapBase(value + 0.5 );
	};
	double min_, max_;
	double colorScale_;
};



#endif /* INCLUDE_FASTFUSION_NODE_VALUETOCOLOR_HPP_ */
