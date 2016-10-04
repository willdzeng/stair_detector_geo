#pragma once
#include <iostream>
#include <utility>

class NeighbourFinder {
public:
	NeighbourFinder() {}

	void start(int x, int y) {
		origin_x_ = x;
		origin_y_ = y;
		current_x_ = x;
		current_y_ = y;
		loop_ = 0;
		bound_x_min_ = origin_x_ - loop_;
		bound_x_max_ = origin_x_ + loop_;
		bound_y_min_ = origin_y_ - loop_;
		bound_y_max_ = origin_y_ + loop_;
		direction_ = 0;
		count_ = 0;
	}

	std::pair<int, int> next() {
		count_ ++;
		bool new_loop = false;

		int x = current_x_;
		int y = current_y_;

		switch (direction_) {
		case 0: x = x + 1; break;
		case 1: y = y + 1; break;
		case 2: x = x - 1; break;
		case 3: y = y - 1; break;
		}

		// std::cout << "count: " << count 
		// << " direction: " << direction_
		// << " x: " << x << " y: " << y
		// << "  x_min:" << bound_x_min_ 
		// << "  x_max: " << bound_x_max_
		// << "  y_min: " << bound_y_min_
		// << "  y_max: " << bound_y_max_ 
		// << std::endl;
		
		// out of right move down and add loop
		if (x > bound_x_max_) {
			new_loop = true;
			direction_ = 1;
		}
		// out of down move left
		if (y > bound_y_max_) {
			y--;
			x--;
			direction_ = 2;
		}
		// out of left move up
		if (x < bound_x_min_) {
			x++;
			y--;
			direction_ = 3;
		}
		// out of up move right
		if (y < bound_y_min_) {
			y++;
			x++;
			direction_ = 0;
		}

		// if new loop update the bound
		if (new_loop) {
			loop_++;
			bound_x_min_ = origin_x_ - loop_;
			bound_x_max_ = origin_x_ + loop_;
			bound_y_min_ = origin_y_ - loop_;
			bound_y_max_ = origin_y_ + loop_;
		}
		// update the current position
		current_x_ = x;
		current_y_ = y;
		// return value
		std::pair<int, int> pos(x, y);
		return pos;
	}

private:
	int origin_x_ = 0;
	int origin_y_ = 0;
	int loop_ = 0;
	int direction_ = 0; // 0 is right, 1 is down, 2 is left, 3 is up
	int current_x_ = 0;
	int current_y_ = 0;
	int bound_x_min_ = 0;
	int bound_x_max_ = 0;
	int bound_y_min_ = 0;
	int bound_y_max_ = 0;
	int count_ = 0;
};
