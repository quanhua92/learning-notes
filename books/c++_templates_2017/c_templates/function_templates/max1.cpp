#include <iostream>
#include <string>
#include <complex>
using namespace std;

template<typename T>
T max(T a, T b) {
	// if b < a then yield a else yield b
	return b < a ? a : b;
};

void main_max1() {
//int main() {
	int i = 42;
	cout << "max(7, i): " << ::max(7, i) << endl;

	double f1 = 3.4;
	double f2 = -6;
	cout << "max(f1, f2): " << ::max(f1, f2) << endl;

	string s1 = "mathematics";
	string s2 = "math";
	cout << "max(s1, s2): " << ::max(s1, s2) << endl;

	//std::complex<float> c1, c2;
	//cout << max(c1, c2); // complex doesn't provide operator < so this will raise error in compile-time

}