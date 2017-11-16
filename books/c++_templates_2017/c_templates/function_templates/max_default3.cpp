#include <iostream>
#include <type_traits>
template<typename T1, typename T2,
	typename RT = std::common_type_t<T1, T2>>
	RT max(T1 a, T2 b)
{
	return b < a ? a : b;
}

//void main() {
void main_default3() {
	std::cout << ::max(40, 10.2) << std::endl;
	std::cout << ::max(5.5, 10.2) << std::endl;
	std::cout << ::max(5.5, 2) << std::endl;
	std::cout << ::max(5.5, 7) << std::endl;

	system("pause");
}
