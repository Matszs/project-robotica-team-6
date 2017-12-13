// a* pathfinding:  https://www.raywenderlich.com/4946/introduction-to-a-pathfinding

#ifndef __SQUARE_H_INCLUDED__
#define __SQUARE_H_INCLUDED__

using namespace std;

class Square {

    private:
    	// Position of square
        int x;
        int y;

        // Scores of A*
        int g = 0; // movement cost // cost from A to square
        int h = 0; // estimated movement cost from the current square to the destination point // estimated cost from square to B

    public:
    	Square(int x, int y, int g, int h);
    	Square(int x, int y);

    	int getX();
    	int getY();

		int getF(); // sum of g + h
		int getG();
		int getH();

		void setG(int value);
		void setH(int value);
};

#endif