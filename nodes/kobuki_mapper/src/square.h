#ifndef __SQUARE_H_INCLUDED__
#define __SQUARE_H_INCLUDED__

using namespace std;

class Square {

    private:
    	// Position of square
        int x;
        int y;

        // Scores of A*
        int g;
        int h;

    public:
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