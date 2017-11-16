#include <iostream>
#include <string>

namespace max4
{
	// maximum of two values of any type
	template<typename T>
	T max(T a, T b) {
		std::cout << "max<T>()" << std::endl;
		return b < a ? a : b;
	}

	// maximum of three values of any type
	template<typename T>
	T max(T a, T b, T c) {
		return max(max(a, b), c); // uses the template version even for ints
		// because the following declaration comes too lates:
		// HHQ-NOTES: This is not true in my experiment within Visual Studio 2017?
	}
	// maximum of two int values
	int max(int a, int b) {
		std::cout << "max(int, int) \n";
		return b < a ? a : b;
	}
}

void main_max4() {
//void main() {
	auto m1 = max4::max(7, 42, 68); // OK
	std::cout << "m1 = " << m1 << std::endl;

	system("pause");
}
