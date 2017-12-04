#include "square.h"

Square::Square(int x, int y) {
	this.x = x;
	this.y = y;
}

Square::Square(int x, int y, int g, int h) {
	this.x = x;
	this.y = y;

	this.g = g;
	this.h = h;
}

int Square::getX() {
	return x;
}

int Square::getY() {
	return y;
}

int Square::getF() {
	return g + h;
}

int Square::getG() {
	return g;
}

int Square::getH() {
	return h;
}

void Square::setG(int value) {
	this.g = value;
}

void Square::setH(int value) {
	this.h = value;
}