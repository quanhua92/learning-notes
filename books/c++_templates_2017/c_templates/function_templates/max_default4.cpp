#include <iostream>

template<typename T1, typename T2>
auto max(T1 a, T2 b) {
	return b < a ? a : b;
}

template<typename RT, typename T1, typename T2>
RT max(T1 a, T2 b) {
	return b < a ? a : b;
}

//void main() {
void main_max_default4() {
	std::cout << ::max(40, 10) << std::endl; // first template
	std::cout << ::max<long double>(5.5, 10.2) << std::endl; // second template

	//auto c = ::max<int>(4, 7.2); // ERROR: both templates match

	system("pause");
}
