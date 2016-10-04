#include <stair_detector_geo/neighbour_finder.h>
#include <iostream>
#include <utility>

int main(){
	NeighbourFinder nf;
	nf.start(0,0);
	for (int i = 0 ; i < 81; i++){
		std::pair<int,int> pos = nf.next();
		std::cout << "x: " << pos.first << "   y: " << pos.second << std::endl;
	}
}