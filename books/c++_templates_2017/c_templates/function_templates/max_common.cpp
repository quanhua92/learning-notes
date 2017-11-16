#include <iostream>
#include <type_traits>
template<typename T1, typename T2>
//std::common_type_t<T1, T2> max(T1 a, T2 b) // C++14
typename std::common_type<T1, T2>::type max(T1 a, T2 b) // C++11
{
	return b < a ? a : b;
}

//void main() {
void main_max_common() {
	auto m = ::max<int, double>(4, 10.2); // specify the template argument list explicitly
	std::cout << m << std::endl;

	std::cout << ::max(40, 10.0) << std::endl;

	system("pause");
}
