#include <iostream>

int max(int a, int b) {
	return b < a ? a : b;
}

template<typename T>
T max(T a, T b)
{
	return b < a ? a : b;
}

//void main() {
void main_max2() {
	std::cout << ::max(40, 10) << std::endl; // nontemplate
	std::cout << ::max(5.5, 10.2) << std::endl; // max<double>
	std::cout << ::max("a", "b") << std::endl; // max<char>
	std::cout << ::max<>(5, 7) << std::endl; // max<int> because of <>
	std::cout << ::max<double>(5.0, 7.2) << std::endl; // max<double> (no argument deduction)
	std::cout << ::max('a', 7.2) << std::endl; // nontemplate

	system("pause");
}
